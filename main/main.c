#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <cJSON.h>

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_tls.h"
#include "mqtt_client.h"

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static const char *TAG = "ESP_MQTTS";

extern const uint8_t ca_cert_pem_start[] asm("_binary_ca_crt_start");
extern const uint8_t ca_cert_pem_end[] asm("_binary_ca_crt_end");
extern const uint8_t client_cert_pem_start[] asm("_binary_client_crt_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_pem_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_pem_end[] asm("_binary_client_key_end");

const char *STATE_POWER_ON = "ON";
const char *STATE_POWER_OFF = "OFF";
const char *MQTT_SET_TOPIC = "discovery/light/esp32_yellow_garland/set";
const char *MQTT_STATE_TOPIC = "discovery/light/esp32_yellow_garland/state";

bool DEFAULT_STATE_POWER = true;
uint8_t DEFAULT_STATE_BRIGHTNESS = 100;

#define WIFI_MAXIMUM_RETRY 15
#define DEFAULT_BUF_SIZE 1024
static int wifi_retry_num = 0;

typedef struct {
    bool power;
    uint8_t brightness;
    uint8_t *effect;
} *esp_state;

static esp_state current_state;

void build_esp_state(esp_state state) {
    state->power = DEFAULT_STATE_POWER;
    state->brightness = DEFAULT_STATE_BRIGHTNESS;
    state->effect = NULL;
}

char * serialize_esp_state(esp_state state) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "uniq_id", "ffffffffffffffff");

    if (state->power) {
        cJSON_AddStringToObject(root, "state", STATE_POWER_ON);
    } else {
        cJSON_AddStringToObject(root, "state", STATE_POWER_OFF);
    }

    cJSON_AddNumberToObject(root, "brightness", state->brightness);

    // TODO: use the map to cast int to string
    if (state->effect) {
        cJSON_AddNumberToObject(root, "effect", *state->effect);
    } else {
        cJSON_AddNullToObject(root, "effect");
    }

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

void deserialize_esp_state(esp_state state, char *json) {
    cJSON *root = cJSON_Parse(json);

    cJSON *power = cJSON_GetObjectItem(root, "state");

    if (power) {
        state->power = strcmp(power->valuestring, STATE_POWER_ON) == 0;
    }

    cJSON *brightness = cJSON_GetObjectItem(root, "brightness");

    if (brightness) {
        state->brightness = brightness->valueint;
    }

    state->effect = NULL;

    cJSON_Delete(root);
}

bool has_esp_state_file() {
    return true;
}

void save_esp_state(esp_state state) {
//    char *json = serialize_esp_state(state);
}

int load_esp_state(esp_state state) {
    bool has = has_esp_state_file();
    if (has) {
        char *json = "{\"brightness\":50}";
        deserialize_esp_state(state, json);
    }

    return 0;
}

void merge_esp_state(esp_state destination, esp_state source) {
    destination->power = source->power;
    destination->brightness = source->brightness;
    destination->effect = source->effect;
}

int apply_esp_state(esp_state state, esp_state applied) {
    if (applied) {
        merge_esp_state(applied, state);
    }

    return EXIT_SUCCESS;
}

int handle_esp_event(char *data) {
    esp_state state = malloc(sizeof(esp_state));
    esp_state applied = malloc(sizeof(esp_state));

    deserialize_esp_state(state, data);
    apply_esp_state(state, applied);
    merge_esp_state(current_state, applied);

    return EXIT_SUCCESS;
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    char topic[DEFAULT_BUF_SIZE];
    char data[DEFAULT_BUF_SIZE];
    esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "[MQTT] Connected");
            esp_mqtt_client_subscribe(client, MQTT_SET_TOPIC, 2);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "[MQTT] Disconnected");
            break;
        case MQTT_EVENT_DATA:
            memset(topic, 0, DEFAULT_BUF_SIZE);
            strncpy(topic, event->topic, event->topic_len);

            memset(data, 0, DEFAULT_BUF_SIZE);
            strncpy(data, event->data, event->data_len);

            ESP_LOGI(TAG, "[MQTT] Data %s with %s", topic, data);

            if (strcmp(topic, MQTT_SET_TOPIC) == 0) {
                handle_esp_event(data);
                char *json = serialize_esp_state(current_state);
                ESP_LOGI(TAG, "[MQTT] Publish data %.*s", strlen(json), json);
                esp_mqtt_client_publish(client, MQTT_STATE_TOPIC, json, 0, 2, 0);
            }
            break;
        default:
            ESP_LOGV(TAG, "[MQTT] Event ID %d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
            .uri = CONFIG_MQTT_URI,
            .event_handle = mqtt_event_handler,
            .cert_pem = (const char *)ca_cert_pem_start,
            .client_cert_pem = (const char *)client_cert_pem_start,
            .client_key_pem = (const char *)client_key_pem_start,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

void wifi_event_handler(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data)
{
    switch(id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "[WIFI] Connecting to %s...", CONFIG_WIFI_SSID);
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "[WIFI] Connected");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            if (wifi_retry_num < WIFI_MAXIMUM_RETRY) {
                ESP_LOGI(TAG, "[WIFI] Reconnecting %d to %s...", wifi_retry_num, CONFIG_WIFI_SSID);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                esp_wifi_connect();
                xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
                wifi_retry_num++;
            }
            break;

        default:
            ESP_LOGI(TAG, "[WIFI] Event base %s with ID %d", base, id);
            break;
    }
}

void ip_event_handler(void* arg, esp_event_base_t base, int32_t id, void* event_data)
{
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "[IP] Got IP:" IPSTR, IP2STR(&event->ip_info.ip));

        wifi_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    } else {
        ESP_LOGI(TAG, "[IP] Event base %s with ID %d", base, id);
    }
}

void wifi_init_sta() {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {
            .sta = {
                    .ssid = CONFIG_WIFI_SSID,
                    .password = CONFIG_WIFI_PASSWORD,
            }
    };

    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    current_state = malloc(sizeof(esp_state));

    build_esp_state(current_state);

    int error = load_esp_state(current_state);
    if (error != 0) {
        ESP_LOGI(TAG, "[APP] Load state error %d", error);
    }

    apply_esp_state(current_state, NULL);

    char *printed_state = serialize_esp_state(current_state);
    ESP_LOGI(TAG, "[APP] Current state: %s", printed_state);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());

    wifi_init_sta();

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

    mqtt_app_start();
}
