
#ifndef sesub_h_
#define sesub_h_

#include "common.h"

#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_FREQ_HZ        400000 // Max 1MHz for esp-idf

/* Declares a function pointer named sensor_reading_f. 
 * The function pointer points to a function that takes a single argument
 * of type sensor_reading_t.
 */
typedef void (*sensor_reading_f)(sensor_reading_t);
/* Declares a function pointer named ambient_alarm_f. 
 * The function pointer points to a function that takes no arguments, indicated by (void).
 */
typedef void (*ambient_alarm_f)(void);

/* Declares a function pointer named ambient_ready_f. 
 * The function pointer points to a function that returns void.
 * The function takes two arguments, both of type float.
 */
typedef void (*ambient_ready_f)(float, float);

typedef struct
{
    int sensor_sda;
    int sensor_scl;

    float temp_high;
    float temp_low;
    float humi_high;
    float humi_low;

    sensor_reading_f new_sensor_reading;
    ambient_alarm_f ambient_alarm;
} sesub_config_t;

void sesub_init(sesub_config_t);
void sesub_start(ambient_ready_f cb);

#endif