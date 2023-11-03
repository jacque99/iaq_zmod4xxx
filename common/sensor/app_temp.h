#ifndef app_temp_h_
#define app_temp_h_

#include "i2c_sensors.h"
#include "driver/temperature_sensor.h"


void init_hs300x(temperature_sensor_handle_t *temp_sensor, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

#endif