#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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

int left_v=200, right_v=-200;




int main(void)
{
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

  //int counter = 0;

  //int sensor_data[8];
    /* Infinite loop. */
    while (1) {
      //waits 1 second
      //int selc = get_selector();
        //front_sensor = VL53L0x_get_dist_mm();
        /*for(int i=0; i<8;i++)
        {
          sensor_data[i] = get_calibrated_prox(i);
        }*/
      char str[100];
      int value=10;
      int str_length=sprintf(str, "Printing number %d!\n",value);
      set_led(LED1,2);
      //set_led(LED3,2);
      e_send_uart1_char(str, str_length);
      chThdSleepMilliseconds(1000);
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
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
