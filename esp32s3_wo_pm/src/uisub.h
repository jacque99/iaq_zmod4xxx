
#ifndef uisub_h_
#define uisub_h_

#include "common.h"

#define LEDSTRIP_GPIO        38
// #define BUTTON_GPIO          39
// #define BUZZER_GPIO          40

typedef void (*button_pressed_f)(void);

typedef struct
{
    int ledstrip_pin;
    int button_pin;
    int buzzer_pin;

    button_pressed_f button_pressed;

    // int oled_sda;
    // int oled_scl;
} uisub_config_t;

void uisub_init(uisub_config_t);
// void uisub_sleep(void);
void uisub_ledstrip(uint32_t, uint32_t, uint32_t);
void uisub_beep(int);
void uisub_show(sensor_reading_t);

#endif
