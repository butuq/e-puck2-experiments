//Perform a LED toggle for the green body of the e-puck2
//by Jahir Argote, march 2021

#include <math.h>
#include "iostream"
#include "leds.h"
#include "spi_comm.h"


int main(void)
{
  spi_comm_start();
  clear_leds();

  while(1)
  {
  set_body_led(2);
  chThdSleepMilliseconds(2000);
  }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
  chSysHalt("Stack smashing detected");
}
