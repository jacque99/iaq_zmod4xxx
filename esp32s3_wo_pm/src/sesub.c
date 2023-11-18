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

#include "uisub.h"
#define TAG "sub.sensors"

static temperature_sensor_handle_t internal_temp_sensor; // FOR TESTING WITH INTERNAL SENSOR 
static hs300x_ctrl_t temp_sensor;
static zmod4xxx_dev_t tvoc_sensor;
static sesub_config_t config;

static ambient_ready_f ambient_cb;

static void read_ambient(void *arg);

/**
 * @brief Sensor subsystem initialization.
 * @param [in] c sesub_config_t struct where provides I2C pins, high and low temperatures for the alarm, 
 * and two callback functions to be called for new sensor readings and alarm cases.
 * @return void
 */
void sesub_init(sesub_config_t c) {

    config = c;

    i2cdev_init();

    init_internal_temp_sensor(&internal_temp_sensor); // FOR TESTING WITH INTERNAL SENSOR 

    /** Temperature and Humidity sensor temp_sensor and TVOC sensor tvoc_sensor share the same I2C pins
     * as provided with the configuration parameter, thus we pass the same pin numbers 
     * as parameters to their corresponding initialization functions.
     */
    memset(&temp_sensor, 0, sizeof(hs300x_ctrl_t));
    temp_sensor.i2c_dev.timeout_ticks = 0xffff / portTICK_PERIOD_MS;
    HS300X_Open(&temp_sensor, HS300X_I2C_ADDR, I2C_MASTER_NUM, (gpio_num_t)c.sensor_sda, (gpio_num_t)c.sensor_scl, I2C_FREQ_HZ);

    memset(&tvoc_sensor, 0, sizeof(zmod4xxx_dev_t));
    tvoc_sensor.i2c_dev.timeout_ticks = 0xffff / portTICK_PERIOD_MS;
    init_tvoc_sensor(&tvoc_sensor, I2C_MASTER_NUM, (gpio_num_t)c.sensor_sda, (gpio_num_t)c.sensor_scl, I2C_FREQ_HZ);
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

/**
 * @brief Reads data from sensors. Checks whether the ambient value exceeds limits, if so calls alarm callback function.
 * Also shares the ambient measurements by using the new_sensor_reading callback function.
 * @param [in] *arg 
 * @return void
 */
static void read_ambient(void *arg)
{
    float temperature, humidity;
    float etoh, tvoc, eco2, iaq;
    // uint32_t presense;

    // int8_t ret;
    rgb_color_t alarm_color;
    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        humidity = 800;
        if (temperature_sensor_get_celsius(internal_temp_sensor, &temperature) == ESP_OK) {
            humidity /= 10;
            float temp_rounded = f_round(temperature, 2);
            ESP_LOGI(TAG, "Temperature value %.02f ℃", temp_rounded);
            if (ambient_cb)
            {
                ambient_cb(temp_rounded, humidity);
            }

            if (temp_rounded > config.temp_high || temp_rounded < config.temp_low) {
                //* Set color to RED 
                alarm_color.red = 255;
                alarm_color.green = 0;
                alarm_color.blue = 0;
            }
            else {
                //* Set color to GREEN 
                alarm_color.red = 0;
                alarm_color.green = 255;
                alarm_color.blue = 0;
            }
            ESP_LOGI(TAG, "RED COLOR %" PRIu32 "", alarm_color.red);
            ESP_LOGI(TAG, "GREEN COLOR %" PRIu32 "", alarm_color.green);
            ESP_LOGI(TAG, "BLUE COLOR %" PRIu32 "", alarm_color.blue);
            if (config.ambient_alarm)
            {
                config.ambient_alarm(alarm_color);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Could not read data from sensor");
        }
        // ret = read_tvoc_sensor(&tvoc_sensor); 
        // if (ret == 0)
        // {
        //     ESP_LOGE(TAG, "Could not read data from TVOC sensor");
        // }
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

/**
 * @brief Starts ambient readings. Creates FreeRTOS task to get readings from the sensors
 * @param [in] cb ambient_ready_f callback function to publish a message to a topic 
 * @return void
 */
void sesub_start(ambient_ready_f cb) {
    ambient_cb = cb;
    /*
     * The function that will be executed when the task is scheduled: read_ambient
     * Size of the task's stack: 5 * configMINIMAL_STACK_SIZE
     * Task parameter:         : NULL No parameters passed
     * The priority of the task: 5
     * The pointer to a variable that will receive the task's handle: NULL don't need the handle
    */
    xTaskCreate(read_ambient, "read", 5 * configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}