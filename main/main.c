#include <string.h>
#include <stdbool.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <cJSON.h>
#include <driver/ledc.h>

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
const char *MQTT_STATUS_TOPIC = "discovery/light/esp32_yellow_garland/status";
const char *MQTT_ATTRS_TOPIC = "discovery/light/esp32_yellow_garland/attributes";

bool DEFAULT_STATE_POWER = true;
int DEFAULT_STATE_BRIGHTNESS = 255;

#define WIFI_MAXIMUM_RETRY 15
#define DEFAULT_BUF_SIZE 1024
#define GPIO_RED_IO 17
#define GPIO_GREEN_IO 18
#define GPIO_BLUE_IO 19
#define GPIO_OUTPUT_PIN_SEL  1ULL<<GPIO_OUTPUT_IO

static int wifi_retry_num = 0;

typedef struct {
    bool *power;
    int *brightness;
    int *red;
    int *green;
    int *blue;
    int *effect;
} esp_state_t;

esp_state_t current_state = {};

ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
};

ledc_channel_config_t ledc_red_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = GPIO_RED_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_FADE_END
};

ledc_channel_config_t ledc_green_channel = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = GPIO_GREEN_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_FADE_END
};

ledc_channel_config_t ledc_blue_channel = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 0,
        .gpio_num   = GPIO_BLUE_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_FADE_END
};

char * serialize_esp_state(esp_state_t *state) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();
//    cJSON_AddNumberToObject(root, "free_heap_size", esp_get_free_heap_size());

    if (state->power) {
        if (*state->power) {
            cJSON_AddStringToObject(root, "state", STATE_POWER_ON);
        } else {
            cJSON_AddStringToObject(root, "state", STATE_POWER_OFF);
            goto end;
        }
    }

    if (state->brightness) {
        cJSON_AddNumberToObject(root, "brightness", *state->brightness);
        cJSON *color = cJSON_AddObjectToObject(root, "color");
        cJSON_AddNumberToObject(color, "r", *state->red);
        cJSON_AddNumberToObject(color, "g", *state->green);
        cJSON_AddNumberToObject(color, "b", *state->blue);
    }

    if (state->effect) {
        cJSON_AddNumberToObject(root, "effect", *state->effect);
    }

    end:
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

void deserialize_esp_state(esp_state_t *state, char *json) {
    cJSON *root = cJSON_Parse(json);

    cJSON *power = cJSON_GetObjectItem(root, "state");
    if (power) {
        if (!state->power) {
            state->power = (bool*)malloc(sizeof(bool));
        }
        *state->power = strcmp(power->valuestring, STATE_POWER_ON) == 0;
    }

    cJSON *brightness = cJSON_GetObjectItem(root, "brightness");
    if (brightness) {
        if (!state->brightness) {
            state->brightness = (int*)malloc(sizeof(int));
        }
        *state->brightness = brightness->valueint;
    }

    cJSON *color = cJSON_GetObjectItem(root, "color");
    if (color) {
        cJSON *red = cJSON_GetObjectItem(color, "r");
        if (red) {
            if (!state->red) {
                state->red = (int*)malloc(sizeof(int));
            }

            *state->red = red->valueint;
        }

        cJSON *green = cJSON_GetObjectItem(color, "g");

        if (green) {
            if (!state->green) {
                state->green = (int*)malloc(sizeof(int));
            }

            *state->green = green->valueint;
        }

        cJSON *blue = cJSON_GetObjectItem(color, "b");

        if (blue) {
            if (!state->blue) {
                state->blue = (int*)malloc(sizeof(int));
            }

            *state->blue = blue->valueint;
        }

    }

    cJSON *effect = cJSON_GetObjectItem(root, "effect");
    if (effect) {
        if (!state->effect) {
            state->effect = (int*)malloc(sizeof(int));
        }
        *state->effect = effect->valueint;
    }

    cJSON_Delete(root);
}

bool has_esp_state_file() {
    return true;
}

esp_err_t load_esp_state(esp_state_t *state) {
    bool has = has_esp_state_file();

    if (has) {
        char *json = "{\"state\":\"ON\",\"brightness\":255,\"color\":{\"r\":0,\"g\":0,\"b\":255}}";
        deserialize_esp_state(state, json);
    } else {
        state->power = (bool*)malloc(sizeof(int));;
        *state->power = DEFAULT_STATE_POWER;
        state->brightness = (int*)malloc(sizeof(int));
        *state->brightness = DEFAULT_STATE_BRIGHTNESS;
    }

    return ESP_OK;
}

esp_err_t apply_esp_state(esp_state_t *state) {
    if (state->power) {
        if (!*state->power) {
            ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, 0);
            ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, 0);
            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, 0);

            ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);
            ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
            return ESP_OK;
        }
    }

    if (state->brightness) {
        double max = 255;
        double c = *state->brightness / max;

        ESP_LOGI(TAG, "C %f", c);
        ESP_LOGI(TAG, "R %f", *state->red * c);
        ESP_LOGI(TAG, "G %f", *state->green * c);
        ESP_LOGI(TAG, "B %f", *state->blue * c);

        ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, (int)(*state->red * c));
        ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, (int)(*state->green * c));
        ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, (int)*state->blue * c);

        ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);
        ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);
        ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
    }

    return ESP_OK;
}

int handle_esp_event(esp_state_t *state, char *data) {
    deserialize_esp_state(state, data);
    apply_esp_state(state);
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
            esp_mqtt_client_publish(client, MQTT_STATUS_TOPIC, "online", 0, 2, 0);
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
                handle_esp_event(&current_state, data);
                char *json = serialize_esp_state(&current_state);
                ESP_LOGI(TAG, "[MQTT] Publish data %.*s", strlen(json), json);
                esp_mqtt_client_publish(client, MQTT_STATE_TOPIC, json, 0, 2, 0);
                free(json);
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

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());

    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_red_channel);
    ledc_channel_config(&ledc_green_channel);
    ledc_channel_config(&ledc_blue_channel);
    ledc_fade_func_install(0);

    esp_err_t error = load_esp_state(&current_state);

    if (error != ESP_OK) {
        ESP_LOGI(TAG, "[APP] Load state error %d", error);
    }

    apply_esp_state(&current_state);

    // TODO: use function to dump data instead of serialize
    char *printed_state = serialize_esp_state(&current_state);
    ESP_LOGI(TAG, "[APP] Current state: %s", printed_state);

//    gpio_config_t io_conf;
//    //disable interrupt
//    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
//    //set as output mode
//    io_conf.mode = GPIO_MODE_OUTPUT;
//    //bit mask of the pins that you want to set,e.g.GPIO18/19
//    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
//    //disable pull-down mode
//    io_conf.pull_down_en = 0;
//    //disable pull-up mode
//    io_conf.pull_up_en = 0;
//    //configure GPIO with the given settings
//    gpio_config(&io_conf);
//    gpio_set_level(GPIO_OUTPUT_IO, true);

    wifi_init_sta();
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    mqtt_app_start();

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // TODO: create internal state
    // update internal state when start / handle / every 5 min
    // publish when update

    // brightness: 1..255
    // color_temp: 153..500
    // white_value: 0..255
    // color: {"r": 0, "g": 255, "b": 0}
}
