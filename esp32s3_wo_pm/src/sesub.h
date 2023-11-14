
#ifndef sesub_h_
#define sesub_h_

#include "common.h"

#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_FREQ_HZ        400000 // Max 1MHz for esp-idf

typedef void (*sensor_reading_f)(sensor_reading_t);
typedef void (*temp_alarm_f)(void);
// temperature and hunidity
typedef void (*temp_ready_f)(float, float);

typedef struct
{
    int sensor_sda;
    int sensor_scl;

    float temp_high;
    float temp_low;

    sensor_reading_f new_sensor_reading;
    temp_alarm_f temp_alarm;
} sesub_config_t;

void sesub_init(sesub_config_t);
void sesub_start(temp_ready_f cb);

#endif