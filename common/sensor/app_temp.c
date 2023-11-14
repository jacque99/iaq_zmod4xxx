#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/temperature_sensor.h" 
#include "app_temp.h"

#define TAG "app.temp"

void init_temp_sensor(temperature_sensor_handle_t *temp_sensor, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t i2c_freq_hz)
{
    ESP_LOGI(TAG, "Install temperature sensor, expected temp ranger range: 10~50 ℃");
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, temp_sensor));

    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(*temp_sensor));
}