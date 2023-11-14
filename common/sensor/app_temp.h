#ifndef app_temp_h_
#define app_temp_h_

#include "hs300x.h"

void init_temp_sensor(temperature_sensor_handle_t *temp_sensor, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t i2c_freq_hz);

#endif