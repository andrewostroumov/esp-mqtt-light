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

/*
 * Description of input values
 * brightness: 1..255
 * color_temp: 153..500
 * white_value: 0..255
 * color: {"r": 0, "g": 255, "b": 0}
 */

static EventGroupHandle_t wifi_event_group, mqtt_event_group, env_event_group, esp_event_group;
static esp_mqtt_client_handle_t client;

static const int CONNECTED_BIT = BIT0;
static const char *TAG = "light";

static const double max_brightness = 255;
static const double min_white =  153;
static const double max_white =  500;
static const double wide = max_white - min_white;

extern const uint8_t ca_cert_pem_start[] asm("_binary_ca_crt_start");
extern const uint8_t ca_cert_pem_end[] asm("_binary_ca_crt_end");

const char *STATE_POWER_ON = "ON";
const char *STATE_POWER_OFF = "OFF";
const char *MQTT_SET_TOPIC = "discovery/light/esp32_rgb_cct_led_strip/set";
const char *MQTT_STATE_TOPIC = "discovery/light/esp32_rgb_cct_led_strip/state";
const char *MQTT_STATUS_TOPIC = "discovery/light/esp32_rgb_cct_led_strip/status";
const char *MQTT_ATTRS_TOPIC = "discovery/light/esp32_rgb_cct_led_strip/attributes";

const char *default_state = "{\"state\":\"ON\",\"brightness\":255,\"white_value\":0,\"color_temp\":153,\"color\":{\"r\":255,\"g\":0,\"b\":0}}";

#define WIFI_MAXIMUM_RETRY 15
#define DEFAULT_BUF_SIZE   1024

#define GPIO_RED_IO        18
#define GPIO_GREEN_IO      19
#define GPIO_BLUE_IO       17
#define GPIO_N_WHITE_IO    23
#define GPIO_W_WHITE_IO    22
#define LED_FADE_MS        300
#define LED_USE_FADE

static int wifi_retry_num = 0;

typedef struct {
    bool *power;
    int *white;
    int *temp;
    int *brightness;
    int *red;
    int *green;
    int *blue;
    int *effect;
} env_state_t;

typedef struct {
    uint32_t free_heap_size;
} esp_state_t;

env_state_t env_state = {};
esp_state_t esp_state = {};

ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
};

ledc_channel_config_t ledc_n_white_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = GPIO_N_WHITE_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_FADE_END
};

ledc_channel_config_t ledc_w_white_channel = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = GPIO_W_WHITE_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_FADE_END
};

ledc_channel_config_t ledc_red_channel = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 0,
        .gpio_num   = GPIO_RED_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_FADE_END
};

ledc_channel_config_t ledc_green_channel = {
        .channel    = LEDC_CHANNEL_3,
        .duty       = 0,
        .gpio_num   = GPIO_GREEN_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_FADE_END
};

ledc_channel_config_t ledc_blue_channel = {
        .channel    = LEDC_CHANNEL_4,
        .duty       = 0,
        .gpio_num   = GPIO_BLUE_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_FADE_END
};

char * serialize_esp_state(esp_state_t* state) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();

    if (state->free_heap_size) {
        cJSON_AddNumberToObject(root, "free_heap_size", state->free_heap_size);
    }

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

void update_esp_state(esp_state_t* state) {
    state->free_heap_size = esp_get_free_heap_size();
}

char * serialize_env_state(env_state_t *state) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();

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

    if (state->white) {
        cJSON_AddNumberToObject(root, "white_value", *state->white);
        cJSON_AddNumberToObject(root, "color_temp", *state->temp);
    }

    if (state->effect) {
        cJSON_AddNumberToObject(root, "effect", *state->effect);
    }

    end:
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

void deserialize_env_state(env_state_t *state, char *json) {
    cJSON *root = cJSON_Parse(json);

    cJSON *power = cJSON_GetObjectItem(root, "state");
    if (power) {
        if (!state->power) {
            state->power = (bool*)malloc(sizeof(bool));
        }
        *state->power = strcmp(power->valuestring, STATE_POWER_ON) == 0;
    }

    cJSON *white = cJSON_GetObjectItem(root, "white_value");
    if (white) {
        if (!state->white) {
            state->white = (int*)malloc(sizeof(int));
        }
        *state->white = white->valueint;
    }

    cJSON *temp = cJSON_GetObjectItem(root, "color_temp");
    if (temp) {
        if (!state->temp) {
            state->temp = (int*)malloc(sizeof(int));
        }
        *state->temp = temp->valueint;
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

bool has_env_state_file() {
    return false;
}

esp_err_t load_env_state(env_state_t *state) {
    bool has = has_env_state_file();

    if (has) {

    } else {
        deserialize_env_state(state, (char*)default_state);
    }

    return ESP_OK;
}

esp_err_t apply_env_state(env_state_t *state) {
    if (state->power) {
        if (!*state->power) {
            ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, 0);
            ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, 0);
            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, 0);
            ledc_set_duty(ledc_green_channel.speed_mode, ledc_n_white_channel.channel, 0);
            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_w_white_channel.channel, 0);

            ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);
            ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_n_white_channel.channel);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_w_white_channel.channel);
            return ESP_OK;
        }
    }

    if (state->brightness) {
        double c;

        if (*state->brightness == 1) {
            c =  0.0;
        } else {
            c = *state->brightness / max_brightness;
        }

#ifdef LED_USE_FADE
        ledc_set_fade_with_time(ledc_red_channel.speed_mode, ledc_red_channel.channel, (int)(*state->red * c), LED_FADE_MS);
        ledc_set_fade_with_time(ledc_green_channel.speed_mode, ledc_green_channel.channel, (int)(*state->green * c), LED_FADE_MS);
        ledc_set_fade_with_time(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, (int)(*state->blue * c), LED_FADE_MS);

        ledc_fade_start(ledc_red_channel.speed_mode, ledc_red_channel.channel, LEDC_FADE_NO_WAIT);
        ledc_fade_start(ledc_green_channel.speed_mode, ledc_green_channel.channel, LEDC_FADE_NO_WAIT);
        ledc_fade_start(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, LEDC_FADE_NO_WAIT);
#else
        ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, (int)(*state->red * c));
        ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, (int)(*state->green * c));
        ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, (int)*state->blue * c);

        ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);
        ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);
        ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
#endif
    }

    if (state->white) {
        int n_white;
        int w_white;

        double c = *state->white / max_brightness;
        double temp = *state->temp - min_white;

        if (temp < (int) (wide / 2) + 1) {
            n_white = (int) max_brightness;
            w_white = (int) ((temp / (int) (wide / 2)) * max_brightness);
        } else {
            w_white = (int) max_brightness;
            n_white = (int) (((wide - temp) / (int) (wide / 2)) * max_brightness);
        }

#ifdef LED_USE_FADE
        ledc_set_fade_with_time(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel, (int)(n_white * c), LED_FADE_MS);
        ledc_set_fade_with_time(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel, (int)(w_white * c), LED_FADE_MS);

        ledc_fade_start(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel, LEDC_FADE_NO_WAIT);
        ledc_fade_start(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel, LEDC_FADE_NO_WAIT);
#else
        ledc_set_duty(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel, (int)(n_white * c));
        ledc_set_duty(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel, (int)(w_white * c));

        ledc_update_duty(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel);
        ledc_update_duty(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel);
#endif
    }

    vTaskDelay(LED_FADE_MS / portTICK_PERIOD_MS);
    return ESP_OK;
}

int handle_env_event(env_state_t *state, char *data) {
    deserialize_env_state(state, data);
    apply_env_state(state);
    return EXIT_SUCCESS;
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    char topic[DEFAULT_BUF_SIZE];
    char data[DEFAULT_BUF_SIZE];

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "[MQTT] Connected");
            xEventGroupSetBits(mqtt_event_group, CONNECTED_BIT);
            esp_mqtt_client_subscribe(event->client, MQTT_SET_TOPIC, 0);
            esp_mqtt_client_publish(event->client, MQTT_STATUS_TOPIC, "online", 0, 0, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            xEventGroupClearBits(mqtt_event_group, CONNECTED_BIT);
            ESP_LOGI(TAG, "[MQTT] Disconnected");
            break;
        case MQTT_EVENT_DATA:
            memset(topic, 0, DEFAULT_BUF_SIZE);
            memset(data, 0, DEFAULT_BUF_SIZE);

            strncpy(topic, event->topic, event->topic_len);
            strncpy(data, event->data, event->data_len);

            ESP_LOGI(TAG, "[MQTT] Data %s with %s", topic, data);

            if (strcmp(topic, MQTT_SET_TOPIC) == 0) {
                handle_env_event(&env_state, data);
                xEventGroupSetBits(env_event_group, CONNECTED_BIT);
                xEventGroupSetBits(esp_event_group, CONNECTED_BIT);
            }
            break;
        default:
            ESP_LOGV(TAG, "[MQTT] Event ID %d", event->event_id);
            break;
    }
    return ESP_OK;
}

void task_env_publish(void *param)
{
    char *json;

    while(true) {
        xEventGroupWaitBits(env_event_group, CONNECTED_BIT, true, true, portMAX_DELAY);
        json = serialize_env_state(&env_state);
        ESP_LOGI(TAG, "[MQTT] Publish data %.*s", strlen(json), json);
        esp_mqtt_client_publish(client, MQTT_STATE_TOPIC, json, 0, 0, 0);
        free(json);
    }
}

void task_esp_publish(void *param)
{
    char *json;

    while(true) {
        xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
        xEventGroupWaitBits(esp_event_group, CONNECTED_BIT, true, true, portMAX_DELAY);
        update_esp_state(&esp_state);
        json = serialize_esp_state(&esp_state);
        ESP_LOGI(TAG, "[MQTT] Publish data %.*s", strlen(json), json);
        esp_mqtt_client_publish(client, MQTT_ATTRS_TOPIC, json, 0, 0, 0);
        free(json);
    }
}

static void mqtt_app_start(void)
{
    mqtt_event_group = xEventGroupCreate();
    env_event_group = xEventGroupCreate();
    esp_event_group = xEventGroupCreate();

    const esp_mqtt_client_config_t mqtt_cfg = {
            .uri = CONFIG_MQTT_URI,
            .event_handle = mqtt_event_handler,
            .cert_pem = (const char *)ca_cert_pem_start,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
    ESP_LOGI(TAG, "[MQTT] Connecting to %s...", CONFIG_MQTT_URI);
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
    ledc_channel_config(&ledc_n_white_channel);
    ledc_channel_config(&ledc_w_white_channel);
    ledc_fade_func_install(0);

    esp_err_t error = load_env_state(&env_state);

    if (error != ESP_OK) {
        ESP_LOGI(TAG, "[APP] Load state error %d", error);
    }

    apply_env_state(&env_state);

    // TODO: use function to dump data instead of serialize
    char *printed_state = serialize_env_state(&env_state);
    ESP_LOGI(TAG, "[APP] Current state: %s", printed_state);

    wifi_init_sta();
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    mqtt_app_start();

    xEventGroupSetBits(esp_event_group, CONNECTED_BIT);
    xTaskCreate(task_env_publish, "task_env_publish", 2048, NULL, 0, NULL);
    xTaskCreate(task_esp_publish, "task_esp_publish", 2048, NULL, 0, NULL);

    // TODO: update internal state every 5 min
    // TODO: use local memory to save state
}
