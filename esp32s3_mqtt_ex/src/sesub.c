#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <bmp280.h>
#include <esp_err.h>
#include <string.h>

#include "sesub.h"
#include "zmod4410_config_iaq2.h"
#include "zmod4xxx.h"
#include "zmod4xxx_hal.h"
#include "iaq_2nd_gen.h"
#include "zmod4xxx_cleaning.h"

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf

static bmp280_t temp_sensor;
static zmod4xxx_dev_t dev;
static sesub_config_t config;

static void read_ambient(void *arg);

int init_zmod4xxx(i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    int8_t ret;
    // zmod4xxx_dev_t dev;
    
    /* Sensor specific variables */
    uint8_t zmod4xxx_status;
    uint8_t track_number[ZMOD4XXX_LEN_TRACKING];
    uint8_t adc_result[ZMOD4410_ADC_DATA_LEN];
    uint8_t prod_data[ZMOD4410_PROD_DATA_LEN];
    iaq_2nd_gen_handle_t algo_handle;
    iaq_2nd_gen_results_t algo_results;
    iaq_2nd_gen_inputs_t algo_input;

    /**** TARGET SPECIFIC FUNCTION ****/
    /*
     * To allow the example running on customer-specific hardware, xthe init_hardware
     * function must be adapted accordingly. The mandatory funtion pointers *read,
     * *write and *delay require to be passed to "dev" (reference files located
     * in "dependencies/zmod4xxx_api/HAL" directory). For more information, read
     * the Datasheet, section "I2C Interface and Data Transmission Protocol".
     */
    ret = init_hardware(&dev);
    if (ret) {
        printf("Error %d during initialize hardware, exiting program!\n", ret);
        goto exit;
    }
    /**** TARGET SPECIFIC FUNCTION ****/
    dev.i2c_dev.port = port;
    dev.i2c_dev.addr = ZMOD4410_I2C_ADDR;
    dev.i2c_dev.cfg.sda_io_num = sda_gpio;
    dev.i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev.i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    ret = i2c_dev_create_mutex(&dev.i2c_dev);
    if (ret) {
        printf("Error %d during initializing sensor, exiting program!\n",
               ret);
        goto exit;
    }
    /* Sensor related data */
    dev.i2c_addr = ZMOD4410_I2C_ADDR;
    dev.pid = ZMOD4410_PID;
    dev.init_conf = &zmod_iaq2_sensor_cfg[INIT];
    dev.meas_conf = &zmod_iaq2_sensor_cfg[MEASUREMENT];
    dev.prod_data = prod_data;

    /* Read product ID and configuration parameters. */
    ret = zmod4xxx_read_sensor_info(&dev);
    if (ret) {
        printf("Error %d during reading sensor information, exiting program!\n",
               ret);
        goto exit;
    }

    /*
     * Retrieve sensors unique tracking number and individual trimming information.
     * Provide this information when requesting support from Renesas.
     */
    ret = zmod4xxx_read_tracking_number(&dev, track_number);
    if (ret) {
        printf("Error %d during reading tracking number, exiting program!\n",
               ret);
        goto exit;
    }
    printf("Sensor tracking number: x0000");
    for (int i = 0; i < sizeof(track_number); i++) {
        printf("%02X", track_number[i]);
    }
    printf("\n");
    printf("Sensor trimming data: ");
    for (int i = 0; i < sizeof(prod_data); i++) {
        printf(" %i", prod_data[i]);
    }
    printf("\n");

    /*
     * Start the cleaning procedure. Check the Programming Manual on indications
     * of usage. IMPORTANT NOTE: The cleaning procedure can be run only once
     * during the modules lifetime and takes 1 minute (blocking).
     */
    printf("Starting cleaning procedure. This might take up to 1 min ...\n");
    ret = zmod4xxx_cleaning_run(&dev);
    if (ERROR_CLEANING == ret) {
        printf("Skipping cleaning procedure. It has already been performed!\n");
    } else if (ret) {
        printf("Error %d during cleaning procedure, exiting program!\n", ret);
        goto exit;
    }

    /* Determine calibration parameters and configure measurement. */
    ret = zmod4xxx_prepare_sensor(&dev);
    if (ret) {
        printf("Error %d during preparation of the sensor, exiting program!\n",
               ret);
        goto exit;
    }

    /*
     * One-time initialization of the algorithm. Handle passed to calculation
     * function.
     */
    ret = init_iaq_2nd_gen(&algo_handle);
    if (ret) {
        printf("Error %d during initializing algorithm, exiting program!\n",
                ret);
        goto exit;
    }

    printf("Evaluate measurements in a loop. Press Ctrl-C to quit.\n\n");
    while ( 1 ) {
        /* Start a measurement. */
        ret = zmod4xxx_start_measurement(&dev);
        if (ret) {
            printf("Error %d during starting measurement, exiting program!\n",
                   ret);
            goto exit;
        }
        /*
         * Perform delay. Required to keep proper measurement timing and keep algorithm accuracy.
         * For more information, read the Programming Manual, section
         * "Interrupt Usage and Measurement Timing".
         */
        dev.delay_ms(ZMOD4410_IAQ2_SAMPLE_TIME);

        /* Verify completion of measurement sequence. */
        ret = zmod4xxx_read_status(&dev, &zmod4xxx_status);
        if (ret) {
            printf("Error %d during reading sensor status, exiting program!\n",
                   ret);
            goto exit;
        }
        /* Check if measurement is running. */
        if (zmod4xxx_status & STATUS_SEQUENCER_RUNNING_MASK) {
            /*
             * Check if reset during measurement occured. For more information,
             * read the Programming Manual, section "Error Codes".
             */
            ret = zmod4xxx_check_error_event(&dev);
            switch (ret) {
            case ERROR_POR_EVENT:
                printf(
                    "Measurement completion fault. Unexpected sensor reset.\n");
                break;
            case ZMOD4XXX_OK:
                printf(
                    "Measurement completion fault. Wrong sensor setup.\n");
                break;
            default:
                printf("Error during reading status register (%d)\n", ret);
                break;
            }
            goto exit;
        }
        /* Read sensor ADC output. */
        ret = zmod4xxx_read_adc_result(&dev, adc_result);
        if (ret) {
            printf("Error %d during reading of ADC results, exiting program!\n",
                   ret);
            goto exit;
        }

        /*
         * Check validity of the ADC results. For more information, read the
         * Programming Manual, section "Error Codes".
         */
        ret = zmod4xxx_check_error_event(&dev);
        if (ret) {
            printf("Error during reading status register (%d)\n", ret);
            goto exit;
        }
        
        /*
         * Assign algorithm inputs: raw sensor data and ambient conditions.
         * Production code should use measured temperature and humidity values.
         */
        algo_input.adc_result = adc_result;
        algo_input.humidity_pct = 50.0;
        algo_input.temperature_degc = 20.0;
        
        /* Calculate algorithm results. */
        ret = calc_iaq_2nd_gen(&algo_handle, &dev, &algo_input, &algo_results); 
        
        printf("*********** Measurements ***********\n");
        for (int i = 0; i < 13; i++) {
            printf(" Rmox[%d] = ", i);
            printf("%.3f kOhm\n", algo_results.rmox[i] / 1e3);
        }
        printf(" Rcda = %.3f kOhm \n", pow(10, algo_results.log_rcda) / 1e3);
        printf(" EtOH = %6.3f ppm\n", algo_results.etoh);
        printf(" TVOC = %6.3f mg/m^3\n", algo_results.tvoc);
        printf(" eCO2 = %4.0f ppm\n", algo_results.eco2);
        printf(" IAQ  = %4.1f\n", algo_results.iaq);

        // /* Check validity of the algorithm results. */
        switch (ret) {
        case IAQ_2ND_GEN_STABILIZATION:
            /* The sensor should run for at least 100 cycles to stabilize.
             * Algorithm results obtained during this period SHOULD NOT be
             * considered as valid outputs! */
            printf("Warm-Up!\n");
            break;
        case IAQ_2ND_GEN_OK:
            printf("Valid!\n");
            break;
        /*
        * Notification from Sensor self-check. For more information, read the
        * Programming Manual, section "Troubleshoot Sensor Damage (Sensor Self-Check)".
        */
        case IAQ_2ND_GEN_DAMAGE:
            printf("Error: Sensor probably damaged. Algorithm results may be incorrect.\n");
            break;
        /* Exit program due to unexpected error. */
        default:
            printf("Unexpected Error during algorithm calculation: Exiting Program.\n");
            goto exit;
        }
        
    };

exit:
    ret = deinit_hardware();
    if (ret) {
        printf("Error %d during deinitializing hardware, exiting program!\n",
               ret);
        return ret;
    }
    return 0;
}

void sesub_init(sesub_config_t c)
{
    config = c;

    i2cdev_init();
    memset(&dev, 0, sizeof(zmod4xxx_dev_t));
    dev.i2c_dev.timeout_ticks = 0xffff / portTICK_PERIOD_MS;
    init_zmod4xxx(I2C_NUM_0, (gpio_num_t)c.sensor_sda, (gpio_num_t)c.sensor_scl);

    memset(&temp_sensor, 0, sizeof(bmp280_t));
    temp_sensor.i2c_dev.timeout_ticks = 0xffff / portTICK_PERIOD_MS;

    bmp280_params_t params;
    bmp280_init_default_params(&params);

    bmp280_init_desc(&temp_sensor, BMP280_I2C_ADDRESS_0, 0, (gpio_num_t)c.sensor_sda, (gpio_num_t)c.sensor_scl);
    bmp280_init(&temp_sensor, &params);
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