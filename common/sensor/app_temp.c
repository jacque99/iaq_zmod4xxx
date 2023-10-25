#include "app_temp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/temperature_sensor.h"

#include "esp_log.h"

#include <stdint.h>

#define TAG "app.temp"

static temp_ready_f temp_cb;

static temperature_sensor_handle_t temp_sensor;

void configure_temperature_sensor(void)
{
    ESP_LOGI(TAG, "Install temperature sensor, expected temp ranger range: 10~50 ℃");
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
}

static void read_temp(void *arg)
{
    int16_t humidity, temperature;
    float tsens_value;

    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        humidity = 1000;
        if (temperature_sensor_get_celsius(temp_sensor, &tsens_value) == ESP_OK)
        {
            humidity /= 10;
            temperature = (int16_t)tsens_value;
            // ESP_LOGI(TAG, "Temperature value %.02f ℃", tsens_value);
            ESP_LOGI(TAG, "Temperature is: %d ℃", temperature);
            if (temp_cb)
            {
                temp_cb(temperature, humidity);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Could not read data from sensor");
        }
    }
}

void apptemp_init(temp_ready_f cb)
{
    temp_cb = cb;
    xTaskCreate(read_temp, TAG, 3 * configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}