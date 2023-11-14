/**
 * @file hs300x.c
 *
 * ESP-IDF driver for Renesas HS300x Temperature and Humidity sensor 
 *
 */

#include <inttypes.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
				   
#include "hs300x.h"
	 
/* Definitions of Open flag */
#define HS300X_OPEN                               (0x433E4432UL) // Open state
	  
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(dev, x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            I2C_DEV_GIVE_MUTEX(&dev->i2c_dev); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

static const char *TAG = "hs300x";

// static esp_err_t read_register16(i2c_dev_t *dev, uint8_t reg, uint16_t *r)
// {
//     uint8_t d[] = { 0, 0 };

//     CHECK(i2c_dev_read_reg(dev, reg, d, 2));
//     *r = d[0] | (d[1] << 8);

//     return ESP_OK;
// }

inline static esp_err_t write_register8(i2c_dev_t *dev, uint8_t addr, uint8_t value)
{
    return i2c_dev_write_reg(dev, addr, &value, 1);
}

/*******************************************************************************************************************//**
 * @brief Opens and configures the HS300X device.
 *
 * Example:
 *
 * @retval ESP_OK                   HS300X successfully configured.
 * @retval ESP_ERR_INVALID_ARG      Invalid I2C address or HS300X is already open.
 * @retval ESP_FAIL                 Null pointer passed as a parameter.
 **********************************************************************************************************************/
esp_err_t HS300X_Open(hs300x_ctrl_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t i2c_freq_hz)
{
    CHECK_ARG(dev);

    if (addr != HS300X_ADDRESS)
    {
        ESP_LOGE(TAG, "Invalid I2C ADDRESS");
        return ESP_ERR_INVALID_ARG;
    }
    if (HS300X_OPEN == dev->open)
    {
        ESP_LOGE(TAG, "HS300X is ALREADY OPEN");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
        dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = i2c_freq_hz;
#endif
    dev->programming_mode.enter = false;

    /* Set open flag */
    dev->open = HS300X_OPEN;
    return i2c_dev_create_mutex(&dev->i2c_dev);
}

/*******************************************************************************************************************//**
 * @brief Disables specified HS300X control. 
 *
 * @retval ESP_OK              Successfully closed.
 * @retval ESP_FAIL                 Null pointer passed as a parameter.
 **********************************************************************************************************************/
esp_err_t HS300X_Close (hs300x_ctrl_t * dev)
{
    CHECK_ARG(dev);

    if (HS300X_OPEN != dev->open)
    {
        ESP_LOGE(TAG, "HS300X is NOT OPEN");
        return ESP_ERR_INVALID_ARG;
    }
    /* Clear Open flag */
    dev->open = 0;
    return i2c_dev_delete_mutex(&dev->i2c_dev);

}

/*******************************************************************************************************************//**
 * @brief This function should be called when start a measurement and when measurement data is stale (khuuchirsan) data.
 * Sends the slave address to the hs300x and start a measurement.
 *
 * @retval ESP_OK                   Successfully started.
 * @retval ESP_FAIL                 Null pointer passed as a parameter.
 **********************************************************************************************************************/
esp_err_t HS300X_MeasurementStart (hs300x_ctrl_t *dev)
{

    dev->buf[0] = 0;
    // dev->buf[0] = HS300X_I2C_ADDRESS << 1;
    ESP_LOGD(TAG, "Measurement Req=%x", dev->buf[0]);
    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, HS300X_ADDRESS, dev->buf[0]), "Failed to start ");

    return ESP_OK;
}

// /**
//  * BMP280 registers
//  */
// #define BMP280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
// #define BMP280_REG_TEMP_LSB    0xFB
// #define BMP280_REG_TEMP_MSB    0xFA
// #define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
// #define BMP280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
// #define BMP280_REG_PRESS_LSB   0xF8
// #define BMP280_REG_PRESS_MSB   0xF7
// #define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
// #define BMP280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
// #define BMP280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
// #define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
// #define BMP280_REG_CTRL_HUM    0xF2 /* bits: 2-0 osrs_h; */
// #define BMP280_REG_RESET       0xE0
// #define BMP280_REG_ID          0xD0
// #define BMP280_REG_CALIB       0x88
// #define BMP280_REG_HUM_CALIB   0x88

// #define BMP280_RESET_VALUE     0xB6

// esp_err_t bmp280_init_default_params(bmp280_params_t *params)
// {
//     CHECK_ARG(params);

//     params->mode = BMP280_MODE_NORMAL;
//     params->filter = BMP280_FILTER_OFF;
//     params->oversampling_pressure = BMP280_STANDARD;
//     params->oversampling_temperature = BMP280_STANDARD;
//     params->oversampling_humidity = BMP280_STANDARD;
//     params->standby = BMP280_STANDBY_250;

//     return ESP_OK;
// }

// esp_err_t bmp280_init(bmp280_t *dev, bmp280_params_t *params)
// {
//     CHECK_ARG(dev && params);

//     I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

//     CHECK_LOGE(dev, i2c_dev_read_reg(&dev->i2c_dev, BMP280_REG_ID, &dev->id, 1), "Sensor not found");

//     if (dev->id != BMP280_CHIP_ID && dev->id != BME280_CHIP_ID)
//     {
//         CHECK_LOGE(dev, ESP_ERR_INVALID_VERSION,
//                 "Invalid chip ID: expected: 0x%x (BME280) or 0x%x (BMP280) got: 0x%x",
//                 BME280_CHIP_ID, BMP280_CHIP_ID, dev->id);
//     }

//     // Soft reset.
//     CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_RESET, BMP280_RESET_VALUE), "Failed to reset sensor");

//     // Wait until finished copying over the NVP data.
//     while (1)
//     {
//         uint8_t status;
//         if (!i2c_dev_read_reg(&dev->i2c_dev, BMP280_REG_STATUS, &status, 1) && (status & 1) == 0)
//             break;
//     }

//     CHECK_LOGE(dev, read_calibration_data(dev), "Failed to read calibration data");

//     if (dev->id == BME280_CHIP_ID)
//     {
//         CHECK_LOGE(dev, read_hum_calibration_data(dev), "Failed to read humidity calibration data");
//     }

//     uint8_t config = (params->standby << 5) | (params->filter << 2);
//     ESP_LOGD(TAG, "Writing config reg=%x", config);

//     CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_CONFIG, config), "Failed to configure sensor");

//     if (params->mode == BMP280_MODE_FORCED)
//     {
//         params->mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
//     }

//     uint8_t ctrl = (params->oversampling_temperature << 5) | (params->oversampling_pressure << 2) | (params->mode);

//     if (dev->id == BME280_CHIP_ID)
//     {
//         // Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
//         uint8_t ctrl_hum = params->oversampling_humidity;
//         ESP_LOGD(TAG, "Writing ctrl hum reg=%x", ctrl_hum);
//         CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_CTRL_HUM, ctrl_hum), "Failed to control sensor");
//     }

//     ESP_LOGD(TAG, "Writing ctrl reg=%x", ctrl);
//     CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_CTRL, ctrl), "Failed to control sensor");

//     I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

//     return ESP_OK;
// }

// esp_err_t bmp280_force_measurement(bmp280_t *dev)
// {
//     CHECK_ARG(dev);

//     I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

//     uint8_t ctrl;
//     I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, BMP280_REG_CTRL, &ctrl, 1));
//     ctrl &= ~0b11;  // clear two lower bits
//     ctrl |= BMP280_MODE_FORCED;
//     ESP_LOGD(TAG, "Writing ctrl reg=%x", ctrl);
//     CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_CTRL, ctrl), "Failed to start forced mode");

//     I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

//     return ESP_OK;
// }

// esp_err_t bmp280_is_measuring(bmp280_t *dev, bool *busy)
// {
//     CHECK_ARG(dev && busy);

//     I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

//     const uint8_t regs[2] = { BMP280_REG_STATUS, BMP280_REG_CTRL };
//     uint8_t status[2];
//     CHECK_LOGE(dev, i2c_dev_read(&dev->i2c_dev, regs, 2, status, 2), "Failed to read status registers");

//     // Check mode - FORCED means BM280 is busy (it switches to SLEEP mode when finished)
//     // Additionally, check 'measuring' bit in status register
//     *busy = ((status[1] & 0b11) == BMP280_MODE_FORCED) || (status[0] & (1 << 3));

//     I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

//     return ESP_OK;
// }


// esp_err_t bmp280_read_fixed(bmp280_t *dev, int32_t *temperature, uint32_t *pressure, uint32_t *humidity)
// {
//     CHECK_ARG(dev && temperature && pressure);

//     int32_t adc_pressure;
//     int32_t adc_temp;
//     uint8_t data[8];

//     // Only the BME280 supports reading the humidity.
//     if (dev->id != BME280_CHIP_ID)
//     {
//         if (humidity)
//             *humidity = 0;
//         humidity = NULL;
//     }

//     I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

//     // Need to read in one sequence to ensure they match.
//     size_t size = humidity ? 8 : 6;
//     CHECK_LOGE(dev, i2c_dev_read_reg(&dev->i2c_dev, 0xf7, data, size), "Failed to read data");

//     adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
//     adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;
//     ESP_LOGD(TAG, "ADC temperature: %" PRIi32, adc_temp);
//     ESP_LOGD(TAG, "ADC pressure: %" PRIi32, adc_pressure);

//     int32_t fine_temp;
//     *temperature = compensate_temperature(dev, adc_temp, &fine_temp);
//     *pressure = compensate_pressure(dev, adc_pressure, fine_temp);

//     if (humidity)
//     {
//         int32_t adc_humidity = data[6] << 8 | data[7];
//         ESP_LOGD(TAG, "ADC humidity: %" PRIi32, adc_humidity);
//         *humidity = compensate_humidity(dev, adc_humidity, fine_temp);
//     }

//     I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

//     return ESP_OK;
// }

// esp_err_t bmp280_read_float(bmp280_t *dev, float *temperature, float *pressure, float *humidity)
// {
//     int32_t fixed_temperature;
//     uint32_t fixed_pressure;
//     uint32_t fixed_humidity;
//     CHECK(bmp280_read_fixed(dev, &fixed_temperature, &fixed_pressure, humidity ? &fixed_humidity : NULL));
//     *temperature = (float)fixed_temperature / 100;
//     *pressure = (float)fixed_pressure / 256;
//     if (humidity)
//         *humidity = (float)fixed_humidity / 1024;

//     return ESP_OK;
// }
