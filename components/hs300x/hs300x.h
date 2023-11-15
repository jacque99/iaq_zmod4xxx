/**
 * @file hs300x.h
 * @defgroup hs300x hs300x
 * @{
 *
 * ESP-IDF driver for Renesas HS300x Temperature and Humidity sensor 
 * 
 */

#ifndef __HS300X_H__
#define __HS300X_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2cdev.h>

#define HS300X_I2C_ADDR  0x44 // The HS300x series default I2C address is 44HEX

/** Data type of HS300X */
typedef enum e_hs300x_data_type
{
    RM_HS300X_HUMIDITY_DATA = 0,
    RM_HS300X_TEMPERATURE_DATA,
} hs300x_data_type_t;

/** Resolution type of HS300X */
typedef enum e_hs300x_resolution
{
    RM_HS300X_RESOLUTION_8BIT  = 0x00,
    RM_HS300X_RESOLUTION_10BIT = 0x04,
    RM_HS300X_RESOLUTION_12BIT = 0x08,
    RM_HS300X_RESOLUTION_14BIT = 0x0C,
} hs300x_resolution_t;

/** HS300X raw data */
typedef struct st_hs300x_raw_data
{
    uint8_t humidity[2];               ///< Upper 2 bits of 0th element are data status
    uint8_t temperature[2];            ///< Lower 2 bits of 1st element are mask
} hs300x_raw_data_t;

/** HS300X sensor data block */
typedef struct st_hs300x_sensor_data
{
    int16_t integer_part;
    int16_t decimal_part;              ///< To two decimal places
} hs300x_sensor_data_t;

/** HS300X data block */
typedef struct st_hs300x_data
{
    hs300x_sensor_data_t humidity;
    hs300x_sensor_data_t temperature;
} hs300x_data_t;

/** HS300X programming mode process block */
typedef struct hs300x_programmnig_mode_params
{
    volatile bool              enter;                  ///< Enter flag.
    volatile bool              blocking;               ///< Blocking flag.
    volatile bool              communication_finished; ///< Communication flag for blocking.
    // volatile rm_hs300x_event_t event;                  ///< Callback event
} hs300x_programmnig_mode_params_t;

/** HS300x Control Block */
typedef struct hs300x_ctrl {
    uint32_t                            open;                 ///< Open flag
    i2c_dev_t                           i2c_dev;              ///< I2C device descriptor
    hs300x_programmnig_mode_params_t programming_mode;     ///< Programming mode flag
    uint8_t buf[3];                                           ///< Buffer for I2C communications
} hs300x_ctrl_t;

/** Open sensor. 
*/
esp_err_t HS300X_Open(hs300x_ctrl_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t i2c_freq_hz);

/** Close HS300X.
*/
esp_err_t HS300X_Close(hs300x_ctrl_t *dev);

/** Start a measurement.
*/
esp_err_t HS300X_MeasurementStart(hs300x_ctrl_t *dev);

/** Read ADC data from HS300X.
 * @param[in]  dev              Pointer to device descriptor.
 * @param[in]  p_raw_data       Pointer to raw data structure.
*/
esp_err_t HS300X_Read(hs300x_ctrl_t *dev, hs300x_raw_data_t * const p_raw_data);

/** Calculate humidity and temperature values from ADC data.
 * @param[in]  dev              Pointer to device descriptor.
 * @param[in]  p_raw_data       Pointer to raw data.
 * @param[in]  p_hs300x_data    Pointer to HS300X data structure.
 */
esp_err_t HS300X_DataCalculate(hs300x_ctrl_t *dev, 
                                  hs300x_raw_data_t * const p_raw_data,
                                  hs300x_data_t * const p_hs300x_data);
/** Enter the programming mode.
*/
esp_err_t HS300X_ProgrammingModeEnter(hs300x_ctrl_t *dev);

/** Change the sensor resolution.
 * @param[in]  dev              Pointer to device descriptor.
 * @param[in]  data_type        Data type of HS300X.
 * @param[in]  resolution       Resolution type of HS300X.
 */
esp_err_t HS300X_ResolutionChange(hs300x_ctrl_t *dev,
                                     hs300x_data_type_t const data_type,
                                     hs300x_resolution_t const resolution);

/** Get the sensor ID.
 * @param[in]  dev              Pointer to device descriptor.
 * @param[in]  p_sensor_id      Pointer to sensor ID of HS300X.
 */
esp_err_t HS300X_SensorIdGet(hs300x_ctrl_t *dev, uint32_t * const p_sensor_id);

/** Exit the programming mode.
* @param[in]  dev              Pointer to device descriptor.
*/
esp_err_t HS300X_ProgrammingModeExit(hs300x_ctrl_t *dev);




// /**
//  * @brief Initialize device descriptor
//  *
//  * @param dev Device descriptor
//  * @param addr BMP280 address
//  * @param port I2C port number
//  * @param sda_gpio GPIO pin for SDA
//  * @param scl_gpio GPIO pin for SCL
//  * @return `ESP_OK` on success
//  */
// esp_err_t bmp280_init_desc(bmp280_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

// /**
//  * @brief Free device descriptor
//  *
//  * @param dev Device descriptor
//  * @return `ESP_OK` on success
//  */
// esp_err_t bmp280_free_desc(bmp280_t *dev);

// /**
//  * @brief Initialize default parameters
//  *
//  * Default configuration:
//  *
//  *  - mode: NORMAL
//  *  - filter: OFF
//  *  - oversampling: x4
//  *  - standby time: 250ms
//  *
//  * @param[out] params Default parameters
//  * @return `ESP_OK` on success
//  */
// esp_err_t bmp280_init_default_params(bmp280_params_t *params);

// /**
//  * @brief Initialize BMP280 module
																				
										   
//  *
//  * Probes for the device, soft resets the device, reads the calibration
//  * constants, and configures the device using the supplied parameters.
//  *
//  * This may be called again to soft reset the device and initialize it again.
//  *
//  * @param dev Device descriptor
//  * @param params Parameters
//  * @return `ESP_OK` on success
//  */
// esp_err_t bmp280_init(bmp280_t *dev, bmp280_params_t *params);

// /**
//  * @brief Start measurement in forced mode
//  *
//  * The module remains in forced mode after this call.
//  * Do not call this method in normal mode.
//  *
//  * @param dev Device descriptor
//  * @return `ESP_OK` on success
//  */
// esp_err_t bmp280_force_measurement(bmp280_t *dev);

// /**
//  * @brief Check if BMP280 is busy
//  *
//  * @param dev Device descriptor
//  * @param[out] busy true if BMP280 measures temperature/pressure
//  * @return `ESP_OK` on success
//  */
// esp_err_t bmp280_is_measuring(bmp280_t *dev, bool *busy);

// /**
//  * @brief Read raw compensated temperature and pressure data
//  *
//  * Temperature in degrees Celsius times 100.
//  *
//  * Pressure in Pascals in fixed point 24 bit integer 8 bit fraction format.
//  *
//  * Humidity is optional and only read for the BME280, in percent relative
//  * humidity as a fixed point 22 bit integer and 10 bit fraction format.
//  *
//  * @param dev Device descriptor
//  * @param[out] temperature Temperature, deg.C * 100
//  * @param[out] pressure Pressure
//  * @param[out] humidity Humidity, optional
//  * @return `ESP_OK` on success
//  */
// esp_err_t bmp280_read_fixed(bmp280_t *dev, int32_t *temperature,
//                             uint32_t *pressure, uint32_t *humidity);

// /**
//  * @brief Read compensated temperature and pressure data
//  *
						
//  * Humidity is optional and only read for the BME280.
//  *
//  * @param dev Device descriptor
//  * @param[out] temperature Temperature, deg.C
//  * @param[out] pressure Pressure, Pascal
//  * @param[out] humidity Relative humidity, percents (optional)
//  * @return `ESP_OK` on success
//  */
// esp_err_t bmp280_read_float(bmp280_t *dev, float *temperature,
//                             float *pressure, float *humidity);

#endif  // __HS300X_H__