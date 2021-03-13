#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "iostream"
#include "leds.h"
#include "spi_comm.h"

//Perform a LED toggle for the green body of the e-puck2
//by Jahir Argote, march 2021

void clear_leds(void);
void spi_comm_start(void);

int main(void)
{
  halInit();
  chSysInit();
  mpu_init();

  clear_leds();
  spi_comm_start();

  while(1)
  {
  set_body_led(2);
  chThdSleepMilliseconds(1000);
  }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
  chSysHalt("Stack smashing detected");
}
