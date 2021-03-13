//Detects nearby objects using sensor data

#include <stdio.h>
#include <math.h>
#include "sensors/proximity.h"
#include "sensors/vL53L0x/VL53L0x.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

proximity_start();
calibrate_ir();
VL53L0x_start();

int sensor_data[8], front_sensor;
int main()
{
  messagebus_init(&bus, &bus_lock, &bus_condvar);
  while(1)
  {
    front_sensor = VL53L0x_get_dist_mm();
    for(int i; i<9;i++)
    {
      sensor_data[i] = get calibrated_prox(i);
    }
  }
}
