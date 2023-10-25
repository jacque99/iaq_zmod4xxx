
#ifndef sesub_h_
#define sesub_h_

#include "common.h"

/*
  This typedef declaration is defining a type called sensor_reading_f, which is a pointer to a function
  that takes one argument of type sensor_reading_t and returns void. 
  This is used for defining function pointers, which can be used to store and call functions dynamically
*/ 
typedef void (*sensor_reading_f)(sensor_reading_t);

/* It defines a type called temp_alarm_f, which is a pointer to a function that takes no arguments (void) and 
  returns void
*/
typedef void (*temp_alarm_f)(void);

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
void sesub_start(void);

#endif