#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>


#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "selector.h"
#include "motors.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "epuck1x/uart/e_uart_char.h"
#include "serial_comm.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int sensor_data[8], front_sensor;
int left_v=0, right_v=0;
float lmt = 0.05;


//Modes of operation
void mode_1();
void mode_2();

//Functions mode 1
void search();    //Move around to explore
void rotate();    //Rotate

//Functions mode 2
void lock_front();
void right_side();
void left_side();
void search2();

//Globals mode1 
int sgn=1, random_dir=1, searchl=300, searchr=500,extra=0,rot_spd=300;
float k = 75;
//Globals mode2
int rot_spd2=200,ref_midpoint = lmt/2,kp=40,ki=10,searchl2=200,searchr2=100;
float acc_errorlef,acc_errorrig,errorlef,errorrig;


int main(void)
{
  //Initialize default
  messagebus_init(&bus, &bus_lock, &bus_condvar);
  halInit();
  chSysInit();
  mpu_init();
  motors_init();
  clear_leds();
  spi_comm_start();

  proximity_start();
  calibrate_ir();
  VL53L0X_start();


  serial_start();
  srand(time(0));
  //int counter = 0;

  //int sensor_data[8];
    /* Infinite loop. */
    while (1) {
      //waits 1 second
      int selc = get_selector();
      //front_sensor = VL53L0x_get_dist_mm();
        
      // Obtain proximity sensor reading
      for(int i=0; i<8;i++)
      {
        sensor_data[i] = get_calibrated_prox(i);
      }
      
      char str[100];
      int value=10;
      int str_length=sprintf(str, "Printing number %d!\n",value);
      set_led(LED1,2);
      e_send_uart1_char(str, str_length); //Send data using bluetooth
      chThdSleepMilliseconds(1000);
      
      //Mode selector
      switch(get_selector())
      {
        case(1): //Explore mode
      {
        mode_1();
        break;
      }
        case(2): //Following mode
      {
        mode_2();
        break;
      }
      default:
        left_v = 0;
        right_v = 0;
        break;
      }

      left_motor_set_speed(left_v);
      right_motor_set_speed(right_v);
      
      /*if (counter < selc)
      {
      set_body_led(2);
      set_led(LED1,2);
      //set_led(LED3,2);
      set_led(LED7,2);
      left_motor_set_speed(left_v);
      right_motor_set_speed(right_v);
      counter++;
      chThdSleepMilliseconds(1000);
      continue;
      }
      left_v = left_v*-1;
      right_v = right_v*-1;
      counter = 0;*/


    }
    return 0;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}


//Mode 1: Explore
void mode_1()
{
  if (sensor_data[1]<lmt || sensor_data[2]<lmt || sensor_data[3]<lmt || sensor_data[4]<lmt)
  {
  	rotate();
  	extra = 0;
  	break;
  }
  	search();
}


//Mode 2: Follow

void mode_2()
{
  static float error1=0,error2=0,acc_errorlef=0,acc_errorrig=0;
  
  search2();
  //Found something by right side
  if (sensor_data[5]<lmt || sensor_data[6]<lmt)
  {
    right_side();
  }      
	//Found something by left side
	if (sensor_data[2]<lmt || sensor_data[1]<lmt)
	{
	  left_side();
	}
	//Found something in the back
	if (sensor_data[7]<lmt || sensor_data[8]<lmt)
	{
	  right_side();
	}
	//Found something in the front
	if (sensor_data[3]<lmt || sensor_data[4]<lmt)
	{
	  lock_front();
	}
}

////////////////////////////////////////////
//Functions mode 1
void search()    //Move around to explore
{
  extra = extra + 0.01;
  left_v = searchl;
  right_v = searchr;
}
void rotate()    //Rotate
{
  left_v = rot_spd;
  right_v = -rot_spd;
}

///////////////////////////////////////////
//Functions mode 2
void lock_front()
{

	//PID controller
	//Error
	errorlef = sensor_data[3] - ref_midpoint;
	errorrig = sensor_data[4] - ref_midpoint;
	acc_errorlef = acc_errorlef + errorlef;
	acc_errorrig = acc_errorrig + errorrig;

	left_v = kp*errorlef + ki*acc_errorlef;
	right_v = kp*errorrig + ki*acc_errorrig;

}
void right_side()
{
  left_v = rot_spd2;
  right_v = -rot_spd2;
  acc_errorlef = 0;
  acc_errorrig = 0;
}

void left_side()
{
  left_v = -rot_spd2;
  right_v = rot_spd2;
  acc_errorlef = 0;
  acc_errorrig = 0;
}
void search2()
{
  left_v = searchl2;
  right_v = searchr2;
  acc_errorlef = 0;
  acc_errorrig = 0;
}


