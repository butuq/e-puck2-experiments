#import "iostream"
#import "leds.h"
#import "spi_comm.h"

//Perform a LED toggle for the green body of the e-puck2
//by Jahir Argote, march 2021

void clear_leds(void);
void spi_comm_start(void);

while(1)
{
  void set_body_led(2);
  void chThdSleepMilliseconds(1000);

}
