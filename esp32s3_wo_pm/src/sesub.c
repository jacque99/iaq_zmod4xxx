#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_log.h>
#include <string.h>
#include <math.h>

#include "sesub.h"
#include "app_temp.h" // FOR TESTING INTERNAL SENSOR
#include "hs300x.h"
#include "app_tvoc.h"

#define TAG "app.sensors"

static temperature_sensor_handle_t internal_temp_sensor; // FOR TESTING INTERNAL SENSOR 
static hs300x_ctrl_t temp_sensor;
static zmod4xxx_dev_t tvoc_sensor;
static sesub_config_t config;

static temp_ready_f temp_cb;
static void read_ambient(void *arg);

void sesub_init(sesub_config_t c) {

    config = c;

    i2cdev_init();

    memset(&tvoc_sensor, 0, sizeof(zmod4xxx_dev_t));
    tvoc_sensor.i2c_dev.timeout_ticks = 0xffff / portTICK_PERIOD_MS;
    init_tvoc_sensor(&tvoc_sensor, I2C_MASTER_NUM, (gpio_num_t)c.sensor_sda, (gpio_num_t)c.sensor_scl, I2C_FREQ_HZ);

    init_internal_temp_sensor(&internal_temp_sensor); // FOR TESTING INTERNAL SENSOR 

    memset(&temp_sensor, 0, sizeof(hs300x_ctrl_t));
    temp_sensor.i2c_dev.timeout_ticks = 0xffff / portTICK_PERIOD_MS;
    HS300X_Open(&temp_sensor, HS300X_I2C_ADDR, I2C_MASTER_NUM, (gpio_num_t)c.sensor_sda, (gpio_num_t)c.sensor_scl, I2C_FREQ_HZ);
}

double f_round(double dval, int n)
{
    char l_fmtp[32], l_buf[64];
    char *p_str;
    sprintf (l_fmtp, "%%.%df", n);
    if (dval>=0)
            sprintf (l_buf, l_fmtp, dval);
    else
            sprintf (l_buf, l_fmtp, dval);
    return ((double)strtod(l_buf, &p_str));

}
static void read_ambient(void *arg)
{
    float temperature, humidity;
    float etoh, tvoc, eco2, iaq;
    // uint32_t presense;

    int8_t ret;
    
    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        humidity = 800;
        if (temperature_sensor_get_celsius(internal_temp_sensor, &temperature) == ESP_OK)
        {
            humidity /= 10;
            float temp_rounded = f_round(temperature, 2);
            ESP_LOGI(TAG, "Temperature value %.02f ℃", temp_rounded);
            if (temp_cb)
            {
                temp_cb(temp_rounded, humidity);
            }

            if (temperature > config.temp_high || temperature < config.temp_low)
            {
                if (config.temp_alarm)
                {
                    config.temp_alarm();
                }
            }
        }
        else
        {
            ESP_LOGE(TAG, "Could not read data from sensor");
        }
        ret = read_tvoc_sensor(&tvoc_sensor); 
        if (ret == 0)
        {
            ESP_LOGE(TAG, "Could not read data from TVOC sensor");
        }
        // presense = 1;
        etoh = 1;
        tvoc = 1;
        eco2 = 1;
        iaq = 1;
        if (config.new_sensor_reading)
        {
            sensor_reading_t reading = {temperature, humidity, //presense, 
                etoh, tvoc, eco2, iaq};
            config.new_sensor_reading(reading);
        }

    }
}

void sesub_start(temp_ready_f cb) {
    temp_cb = cb;
    xTaskCreate(read_ambient, "read", 5 * configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
