
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <stdio.h>
#include <esp_log.h>
#include <string.h>

#include <driver/gpio.h>

#include "led_strip.h"
#include "uisub.h"

#define TAG "sub.ui"

#define SHOW_TEMPERATURE 0
#define SHOW_HUMIDITY 1

static uisub_config_t config;
// static SemaphoreHandle_t disp_guard;
static sensor_reading_t last_reading;

static void configure_led(void);
static void reset_display(void);
static void beep(void *arg);

static int button_state = 0;
static led_strip_handle_t led_strip;
static uint8_t s_led_state = 0;

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configure addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = LEDSTRIP_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

void uisub_init(uisub_config_t c)
{
    config = c;

    //* Configure Button */

    //* Configure the peripheral according to the LED type */
    configure_led();
    s_led_state = 1; // Enables LED

    //* Configure Buzzer pin */
    // gpio_config_t io_conf;
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // io_conf.pin_bit_mask = (1ULL << config.buzzer_pin);
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // gpio_config(&io_conf);

    //* Initialize Display */
    // init_display();
    // disp_guard = xSemaphoreCreateMutex();
    reset_display();
}

void uisub_ledstrip(uint32_t red, uint32_t green, uint32_t blue)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, red, green, blue);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void uisub_show(sensor_reading_t data)
{
    // if (xSemaphoreTake(disp_guard, 0) == pdFALSE)
    // {
    //     return;
    // }
    reset_display();
    char buff[10];

    switch ((button_state / 4) % 4)
    {
    case SHOW_TEMPERATURE:
        sprintf(buff, "%.02f ℃", data.temperature);
        break;
    case SHOW_HUMIDITY:
        sprintf(buff, "%.02f", data.humidity);
        break;
    default:
        break;
    }

    last_reading = data;

    // xSemaphoreGive(disp_guard);
}

static void reset_display(void)
{
    uint16_t reset_display=1;
    reset_display++;
}

static void beep(void *arg)
{
    int cnt = 2 * (int)arg;
    bool state = true;
    for (int i = 0; i < cnt; ++i, state = !state)
    {
        gpio_set_level((gpio_num_t)config.buzzer_pin, state);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void uisub_beep(int cnt)
{
    xTaskCreate(beep, "beep", 3 * configMINIMAL_STACK_SIZE, (void *)cnt, 5, NULL);
}