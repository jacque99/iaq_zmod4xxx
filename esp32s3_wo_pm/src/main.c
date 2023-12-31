/* ESP32-S3 Indoor Air Quality monitoring device
   ThingsBoard MQTT SSL Mutual Authentication

*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include <cJSON.h>

#include "sesub.h"
#include "uisub.h"
#include "app_wifi.h"

static const char *TAG = "IAQ_STA";

extern const uint8_t client_cert_pem_start[] asm("_binary_cert_pem_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_cert_pem_end");
extern const uint8_t client_key_pem_start[] asm("_binary_key_pem_start");
extern const uint8_t client_key_pem_end[] asm("_binary_key_pem_end");
extern const uint8_t server_cert_pem_start[] asm("_binary_server_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_server_pem_end");

static esp_mqtt_client_handle_t client = NULL;

static void publish_reading(float temp, float hum)
{
    char *json_str;

    /* create a json message */
    cJSON *response;
    response = cJSON_CreateObject();
    char print_num[18];
    snprintf(print_num, 18, "%.2f", temp);
    // cJSON_AddRawToObject(jobj, "field_name", print_num);
    cJSON_AddRawToObject(response, "temperature", print_num);

    json_str = cJSON_Print(response);

    if (client != NULL)
    {
        int msg_id = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", json_str, 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    static int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_subscribe(client, "v1/devices/me/telemetry", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
  const esp_mqtt_client_config_t mqtt_cfg = {
    .broker = {
        .address.uri = CONFIG_BROKER_URI,
        .verification.certificate = (const char *)server_cert_pem_start
    },
    .credentials = {
      .authentication = {
        .certificate = (const char *)client_cert_pem_start,
        .key = (const char *)client_key_pem_start,
      },
    }
  };

    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

/*
 * @brief Wi-Fi callback function, when connected
 *
 *  This function is passed as an argument to appwifi_connect and intended to be called when connected.
 */
static void handle_wifi_connect(void)
{
    mqtt_app_start();
    sesub_start(publish_reading);
}

/*
 * @brief Wi-Fi callback function, when connection failed
 *
 *  This function is passed as an argument to appwifi_connect and intended to be called when connection failed.
 */
static void handle_wifi_failed(void)
{
    ESP_LOGE(TAG, "wifi failed");
}

static void update_power_man(void)
{
    uint16_t a=1;
    a++;
    // pmsub_update(false);
}

static void ambient_alarm(rgb_color_t alarm_color)
{
    uisub_ledstrip(alarm_color.red, alarm_color.green, alarm_color.blue);
}

static void init_subsystems(void)
{
    sesub_config_t se_cfg = {
        .sensor_sda = SENSOR_BUS_SDA,
        .sensor_scl = SENSOR_BUS_SCL,
        .temp_high = 35,
        .temp_low = 10,
        .new_sensor_reading = uisub_show,
        .ambient_alarm = ambient_alarm, 
    };
    sesub_init(se_cfg);

    uisub_config_t ui_cfg = {
        .ledstrip_pin = LEDSTRIP_GPIO,
        // .button_pin = BUTTON_GPIO,
        // .buzzer_pin = BUZZER_GPIO,

        .button_pressed = update_power_man,

        // .oled_sda = OLED_SDA,
        // .oled_scl = OLED_SCL,
    };
    uisub_init(ui_cfg);    
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize the sensor subsystems */
    init_subsystems();

    /* The app_main function calls appwifi_connect with callbacks.
       It reads Wi-Fi credentials, as indicated in menuconfig.
     */

    connect_wifi_params_t cbs = {
            .on_connected = handle_wifi_connect,
            .on_failed = handle_wifi_failed};
    appwifi_connect(cbs);
}