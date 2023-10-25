#ifndef app_temp_h_
#define app_temp_h_

// temperature 
typedef void (*temp_ready_f)(int, int);

void configure_temperature_sensor(void);
void apptemp_init(temp_ready_f cb);


#endif