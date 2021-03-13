//Controls the movement of the agent with a selector switch

#include <stdio.h>
#include <math.h>
#include "selector.h"
#include "motors.h"


void movement_1();
void movement_2();

int left_v, right_v;

int main()
{
  motor_init();
  while(1)
  {
    //Select mode of operation
    switch(get_selector())
    {
      case(1): //Forward movement
      {
        movement_1();
        break;
      }
      case(2): //Rotation movement
      {
        movement_2();
        break;
      }
      default:
        break;
    }
    //Set motor velocities
    left_motor_set_speed(left_v);
    right_motor_set_speed(right_v);
  }
}

void movement_1()
{
  left_v = 500;
  right_v = 500;
}

void movement_2()
{
  left_v = -500;
  right_v = 500;
}

