//Task 1: E-puck robot that explores a small enviorment and perform collision avoidance.

#include "epuck1x/uart/e_uart_char.h"
#include "stdio.h"
#include "serial_comm.h"
#include "leds.h"
#include "spi_comm.h"
#include "sensors/VL53L0x/VL53L0x.h"
#include "sensors/proximity.h"
#include "epuck1x/uart/e_uart_char.h"
#include "selector.h"
#include "motors.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#define MAX_RANGE 1200

serial_start();
messagebus_t bus();
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_lock);

int prox_sensor[8],front_sensor,v_left,v_right,rnum;
bool front_collision,back_collision,left_collision,right_collision;
void read_prox_sensor(void);
void read_frontal_sensor(void);
void mode_1();
void mode_2();
void base_mode();

clear_leds();
spi_comm_start();
proximity_start();
calibrate_ir();
motors_init();

int main()
{
  messagebus_init(&bus, &bus_lock, &bus_condvar);
  srand (time(NULL));
  char str[100];
  int str_length;
  str_length = sprintf(str, "Hello world\n");
    
  while(1)
  {
    e_send_uart1_char(str, str_length); //Send Bluetooth data
    read_prox_sensor(); //Reads IR sensors
    read_frontal_sensor(); //Read frontal sensor
    
    switch(get_selector())
    {
      case(1):
        mode_1();
        break;
      case(2):
        mode_2();
        break;
      default:
        base_mode();
        break;

    }
    //Limit Left and Right speeds
    if(v_left<-1000)
      v_left = -1000;
    else if (v_left>1000)
      v_left = 1000;

    if(v_right<-1000)
      v_right = -1000;
    else if (v_right>1000)
      v_right = 1000;

    //Action signals
    left_motor_set_speed(v_left);
    right_motor_set_speed(v_right);
  }
}

void read_prox_sensor()
{
  for (int i=0; i<9; i++)
    prox_sensor[i] = get_calibrated_prox(i);
}

void read_frontal_sensor()
{
  front_sensor = VL53L0x_get_dist_mm();
}

//Search area and collision avoidance
void mode_1()
{

  rnun = rand();
}

//Follow near object
void mode_2()
{
  
}

//For testing
void base_mode()
{
  v_left = 500;
  v_right = -500;
}

void collision_handler()
{
  
  if(prox_sensor[0] > MAX_RANGE  && prox_sensor[7] > MAX_RANGE)
    front_collision == true;
  else
    front_collision == false;
  if(prox_sensor[5] > MAX_RANGE  && prox_sensor[6] > MAX_RANGE)
    left_collision == true;
  else
    left_collision == false;
  if(prox_sensor[1] > MAX_RANGE && prox_sensor[2] > MAX_RANGE)
    right_collision == true;
  else
    right_collision == false;
  if(prox_sensor[3] > MAX_RANGE && prox_sensor[4] > MAX_RANGE)
    back_collision == true;
  else
    back_collision == false;



}
