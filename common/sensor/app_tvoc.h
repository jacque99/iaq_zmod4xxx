#ifndef app_tvoc_h_
#define app_tvoc_h_

#include "zmod4xxx.h"

int read_tvoc_sensor(zmod4xxx_dev_t *dev);
int init_tvoc_sensor(zmod4xxx_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t i2c_freq_hz);

#endif