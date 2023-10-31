#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <bmp280.h>
#include <esp_err.h>
#include <string.h>

#include "sesub.h"
#include "app_tvoc.h"
#include "zmod4xxx.h"

static bmp280_t temp_sensor;
static zmod4xxx_dev_t tvoc_sensor;
static sesub_config_t config;

static void read_ambient(void *arg);

void sesub_init(sesub_config_t c) {

    config = c;

    i2cdev_init();
    memset(&tvoc_sensor, 0, sizeof(zmod4xxx_dev_t));
    tvoc_sensor.i2c_dev.timeout_ticks = 0xffff / portTICK_PERIOD_MS;
    init_zmod4xxx(&tvoc_sensor, I2C_NUM_0, (gpio_num_t)c.sensor_sda, (gpio_num_t)c.sensor_scl);

    // memset(&temp_sensor, 0, sizeof(bmp280_t));
    // temp_sensor.i2c_dev.timeout_ticks = 0xffff / portTICK_PERIOD_MS;

    // bmp280_params_t params;
    // bmp280_init_default_params(&params);

    // bmp280_init_desc(&temp_sensor, BMP280_I2C_ADDRESS_0, 0, (gpio_num_t)c.sensor_sda, (gpio_num_t)c.sensor_scl);
    // bmp280_init(&temp_sensor, &params);
}

void sesub_start(void) {
    xTaskCreate(read_ambient, "read", 5 * configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}

static void read_ambient(void *arg)
{
    float pressure, temperature, humidity;
    float etoh, eco2, tvoc, iaq;
    // uint32_t presense;

    while (1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(bmp280_read_float(&temp_sensor, &temperature, &pressure, &humidity));
        // ESP_ERROR_CHECK(tsl2561_read_lux(&light_sensor, &lux));
        if (temperature > config.temp_high || temperature < config.temp_low)
        {
            if (config.temp_alarm)
            {
                config.temp_alarm();
            }
        }
        // presense = 1;
        etoh = 1;
        eco2 = 1;
        tvoc = 1;
        iaq = 1;
        if (config.new_sensor_reading)
        {
            sensor_reading_t reading = {(int)temperature, (int)humidity, //(int)(presense), 
                (int)(etoh), (int)(eco2), (int)(tvoc), (int)(iaq)};
            config.new_sensor_reading(reading);
        }
    }
}