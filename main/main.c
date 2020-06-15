#include <string.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <cJSON.h>
#include <driver/ledc.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_tls.h"
#include "mqtt_client.h"
#include "esp_discovery.h"

/*
 * Description of input values
 * brightness: 1..255
 * color_temp: 153..500
 * white_value: 0..255
 * color: {"r": 0, "g": 255, "b": 0}
 */

#define SW_VERSION "2.0.1"

#define DEFAULT_BUF_SIZE   1024

#define GPIO_RED_IO        GPIO_NUM_19
#define GPIO_GREEN_IO      GPIO_NUM_22
#define GPIO_BLUE_IO       GPIO_NUM_17
#define LED_FADE_MS        300
#define LED_USE_FADE
//#define LED_HAS_WHITE

#ifdef LED_HAS_WHITE
#define GPIO_N_WHITE_IO    GPIO_NUM_23
#define GPIO_W_WHITE_IO    GPIO_NUM_22
#endif

#define STORAGE_NAMESPACE  "storage"
#define STORAGE_LED_KEY "led"
#define STORAGE_SERIAL_KEY "serial"
#define STORAGE_SERIAL_LENGTH 8

#define PERSIST_DELAY 60000

static EventGroupHandle_t wifi_event_group, mqtt_event_group, env_event_group, esp_event_group, persist_event_group, discovery_event_group;
static esp_mqtt_client_handle_t client;

static const int CONNECTED_BIT = BIT0;
static const int DISCOVERY_BIT = BIT1;
static const char *TAG = "light";

static const double max_brightness = 255;

#ifdef LED_HAS_WHITE
static const double min_white = 153;
static const double max_white = 500;
static const double wide = max_white - min_white;
#endif

extern const uint8_t ca_cert_pem_start[] asm("_binary_ca_crt_start");
extern const uint8_t ca_cert_pem_end[] asm("_binary_ca_crt_end");

const char *STATE_POWER_ON = "ON";
const char *STATE_POWER_OFF = "OFF";

static const char *STATUS_ONLINE = "online";
static const char *STATUS_OFFLINE = "offline";

const char *default_state = "{\"state\":\"ON\",\"brightness\":255,\"white_value\":0,\"color_temp\":153,\"color\":{\"r\":255,\"g\":0,\"b\":0}}";

static uint8_t serial_data[STORAGE_SERIAL_LENGTH];

typedef enum {
    STATE_CHANGE_NULL,
    STATE_CHANGE_WHITE,
    STATE_CHANGE_COLOR,
    STATE_CHANGE_EFFECT,
} env_state_change_t;

typedef struct {
    bool *power;
    int *white;
    int *temp;
    int *brightness;
    int *red;
    int *green;
    int *blue;
    int *effect;
    env_state_change_t last_change;
} env_state_t;

typedef struct {
    uint32_t free_heap_size;
} esp_state_t;

env_state_t env_state = {};
esp_state_t esp_state = {};

esp_serial_t esp_serial = {
        .namespace = STORAGE_NAMESPACE,
        .key = STORAGE_SERIAL_KEY,
        .data = serial_data,
        .length = STORAGE_SERIAL_LENGTH
};

esp_device_t esp_device = {
        .sw_version = SW_VERSION
};

esp_discovery_t esp_discovery = {
        .esp_serial  = &esp_serial,
        .esp_device  = &esp_device,
        .platform    = "mqtt",
        .schema      = "json",
        .min_mireds  = 153,
        .max_mireds  = 588,
        .brightness  = true,
        .color_temp  = true,
        .white_value = false,
        .rgb         = true,
        .effect      = false,
        .retain      = false,
        .optimistic  = false,
};

ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
};

#ifdef LED_HAS_WHITE

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

#endif

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

char *serialize_esp_state(esp_state_t *state) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();

    if (state->free_heap_size) {
        cJSON_AddNumberToObject(root, "free_heap_size", state->free_heap_size);
    }

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

void update_esp_state(esp_state_t *state) {
    state->free_heap_size = esp_get_free_heap_size();
}

void mired_env_state(env_state_t *state) {
    if (!state->temp) { return; }

    double temp = 10000 / (double) *state->temp; //kelvins = 1,000,000/mired (and that /100)

    if (!state->red) {
        state->red = (int *) malloc(sizeof(int));
    }

    if (!state->green) {
        state->green = (int *) malloc(sizeof(int));
    }

    if (!state->blue) {
        state->blue = (int *) malloc(sizeof(int));
    }

    if (temp <= 66) {
        *state->red = 255;
        *state->green = (int) (99.470802 * log(temp) - 161.119568);

        if (temp <= 19) {
            *state->blue = 0;
        } else {
            *state->blue = (int) (138.517731 * log(temp - 10) - 305.044793);
        }
    } else {
        *state->red = (int) (329.698727 * pow(temp - 60, -0.13320476));
        *state->green = (int) (288.12217 * pow(temp - 60, -0.07551485));
        *state->blue = 255;
    }
}

char *serialize_env_state(env_state_t *state) {
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
    }

    if (state->temp) {
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
            state->power = (bool *) malloc(sizeof(bool));
        }
        *state->power = strcmp(power->valuestring, STATE_POWER_ON) == 0;
    }

    cJSON *white = cJSON_GetObjectItem(root, "white_value");
    if (white) {
        if (!state->white) {
            state->white = (int *) malloc(sizeof(int));
        }
        *state->white = white->valueint;
    }

    cJSON *temp = cJSON_GetObjectItem(root, "color_temp");
    if (temp) {
        if (!state->temp) {
            state->temp = (int *) malloc(sizeof(int));
        }
        *state->temp = temp->valueint;
        state->last_change = STATE_CHANGE_WHITE;
    }

    cJSON *brightness = cJSON_GetObjectItem(root, "brightness");
    if (brightness) {
        if (!state->brightness) {
            state->brightness = (int *) malloc(sizeof(int));
        }
        *state->brightness = brightness->valueint;
    }

    cJSON *color = cJSON_GetObjectItem(root, "color");
    if (color) {
        cJSON *red = cJSON_GetObjectItem(color, "r");
        if (red) {
            if (!state->red) {
                state->red = (int *) malloc(sizeof(int));
            }

            *state->red = red->valueint;
        }

        cJSON *green = cJSON_GetObjectItem(color, "g");

        if (green) {
            if (!state->green) {
                state->green = (int *) malloc(sizeof(int));
            }

            *state->green = green->valueint;
        }

        cJSON *blue = cJSON_GetObjectItem(color, "b");

        if (blue) {
            if (!state->blue) {
                state->blue = (int *) malloc(sizeof(int));
            }

            *state->blue = blue->valueint;
        }

        state->last_change = STATE_CHANGE_COLOR;
    }

    cJSON *effect = cJSON_GetObjectItem(root, "effect");
    if (effect) {
        if (!state->effect) {
            state->effect = (int *) malloc(sizeof(int));
        }
        *state->effect = effect->valueint;
    }

    cJSON_Delete(root);
}

esp_err_t load_env_state(env_state_t *state) {
    esp_err_t err;
    nvs_handle_t nvs;
    uint8_t *data;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    size_t required_size = 0;
    err = nvs_get_blob(nvs, STORAGE_LED_KEY, NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    if (required_size) {
        data = malloc(required_size);
        err = nvs_get_blob(nvs, STORAGE_LED_KEY, data, &required_size);
        if (err != ESP_OK) return err;
        deserialize_env_state(state, (char *) data);
    } else {
        deserialize_env_state(state, (char *) default_state);
    }

    nvs_close(nvs);
    return ESP_OK;
}

esp_err_t save_env_state(env_state_t *state) {
    esp_err_t err;
    nvs_handle_t nvs;
    char *data;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    data = serialize_env_state(state);
    err = nvs_set_blob(nvs, STORAGE_LED_KEY, data, strlen(data));
    if (err != ESP_OK) return err;

    nvs_close(nvs);
    return ESP_OK;
}

esp_err_t apply_env_state(env_state_t *state) {
    if (state->power) {
        if (!*state->power) {
            ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, 0);
            ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, 0);
            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, 0);

            ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);
            ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);

#ifdef LED_HAS_WHITE

            ledc_set_duty(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel, 0);
            ledc_set_duty(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel, 0);

            ledc_update_duty(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel);
            ledc_update_duty(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel);

#endif
            return ESP_OK;
        }
    }


    if (state->brightness) {
        double c;
        double r, g, b;

        c = *state->brightness / max_brightness;
        r = *state->red * c;
        g = *state->green * c;
        b = *state->blue * c;

#ifdef LED_USE_FADE
        ledc_set_fade_with_time(ledc_red_channel.speed_mode, ledc_red_channel.channel, (int) r,
                                LED_FADE_MS);
        ledc_set_fade_with_time(ledc_green_channel.speed_mode, ledc_green_channel.channel, (int) g,
                                LED_FADE_MS);
        ledc_set_fade_with_time(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, (int) b,
                                LED_FADE_MS);

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

#ifdef LED_HAS_WHITE

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
        ledc_set_fade_with_time(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel, (int) (n_white * c),
                                LED_FADE_MS);
        ledc_set_fade_with_time(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel, (int) (w_white * c),
                                LED_FADE_MS);

        ledc_fade_start(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel, LEDC_FADE_NO_WAIT);
        ledc_fade_start(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel, LEDC_FADE_NO_WAIT);
#else
        ledc_set_duty(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel, (int)(n_white * c));
        ledc_set_duty(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel, (int)(w_white * c));

        ledc_update_duty(ledc_n_white_channel.speed_mode, ledc_n_white_channel.channel);
        ledc_update_duty(ledc_w_white_channel.speed_mode, ledc_w_white_channel.channel);
#endif

    }

#endif

    vTaskDelay(LED_FADE_MS / portTICK_PERIOD_MS);
    return ESP_OK;
}

int handle_env_event(env_state_t *state, char *data) {
    deserialize_env_state(state, data);

#ifndef LED_HAS_WHITE
    if (state->last_change == STATE_CHANGE_WHITE) {
        mired_env_state(state);
    }
#endif

    apply_env_state(state);
    return EXIT_SUCCESS;
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    char topic[DEFAULT_BUF_SIZE];
    char data[DEFAULT_BUF_SIZE];

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "[MQTT] Connected");
            esp_mqtt_client_subscribe(event->client, esp_discovery.set_topic, 0);
            esp_mqtt_client_publish(event->client, esp_discovery.status_topic, STATUS_ONLINE, 0, 0, true);
            xEventGroupSetBits(mqtt_event_group, CONNECTED_BIT);
            xEventGroupSetBits(discovery_event_group, CONNECTED_BIT);
            xEventGroupSetBits(env_event_group, CONNECTED_BIT);
            xEventGroupSetBits(esp_event_group, CONNECTED_BIT);
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

            if (strcmp(topic, esp_discovery.set_topic) == 0) {
                handle_env_event(&env_state, data);
                xEventGroupSetBits(persist_event_group, CONNECTED_BIT);
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

void task_env_persist(void *param) {
    while (true) {
        xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
        xEventGroupWaitBits(persist_event_group, CONNECTED_BIT, true, true, portMAX_DELAY);
        save_env_state(&env_state);
        vTaskDelay(PERSIST_DELAY / portTICK_RATE_MS);
    }
}

void task_env_publish(void *param) {
    char *json;

    while (true) {
        xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT | DISCOVERY_BIT, false, true, portMAX_DELAY);
        xEventGroupWaitBits(env_event_group, CONNECTED_BIT, true, true, portMAX_DELAY);
        json = serialize_env_state(&env_state);
        ESP_LOGI(TAG, "[MQTT] Publish topic %s data %.*s", esp_discovery.state_topic, strlen(json), json);
        esp_mqtt_client_publish(client, esp_discovery.state_topic, json, 0, 0, true);
        free(json);
    }
}

void task_esp_publish(void *param) {
    char *json;

    while (true) {
        xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT | DISCOVERY_BIT, false, true, portMAX_DELAY);
        xEventGroupWaitBits(esp_event_group, CONNECTED_BIT, true, true, portMAX_DELAY);
        update_esp_state(&esp_state);
        json = serialize_esp_state(&esp_state);
        ESP_LOGI(TAG, "[MQTT] Publish topic %s data %.*s", esp_discovery.attributes_topic, strlen(json), json);
        esp_mqtt_client_publish(client, esp_discovery.attributes_topic, json, 0, 0, false);
        free(json);
    }
}

void task_discovery_publish(void *param) {
    char *json;

    while (true) {
        xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
        xEventGroupWaitBits(discovery_event_group, CONNECTED_BIT, true, true, portMAX_DELAY);
        json = esp_discovery_serialize(&esp_discovery);
        ESP_LOGI(TAG, "[MQTT] Publish topic %s data %.*s", esp_discovery.discovery_topic, strlen(json), json);
        esp_mqtt_client_publish(client, esp_discovery.discovery_topic, json, 0, 0, true);
        xEventGroupSetBits(mqtt_event_group, DISCOVERY_BIT);
        free(json);
    }
}

static void mqtt_app_start(void) {
    mqtt_event_group = xEventGroupCreate();
    env_event_group = xEventGroupCreate();
    esp_event_group = xEventGroupCreate();
    discovery_event_group = xEventGroupCreate();
    persist_event_group = xEventGroupCreate();

    const esp_mqtt_client_config_t mqtt_cfg = {
            .uri = CONFIG_MQTT_URI,
            .username = CONFIG_MQTT_USERNAME,
            .password = CONFIG_MQTT_PASSWORD,
            .event_handle = mqtt_event_handler,
            .cert_pem = (const char *) ca_cert_pem_start,
            .lwt_topic = esp_discovery.status_topic,
            .lwt_msg = STATUS_OFFLINE,
            .lwt_qos = 0,
            .lwt_retain = true,
            .keepalive = 10
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
    ESP_LOGI(TAG, "[MQTT] Connecting to %s...", CONFIG_MQTT_URI);
}

void wifi_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data) {
    switch (id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "[WIFI] Connecting to %s...", CONFIG_WIFI_SSID);
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "[WIFI] Connected");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "[WIFI] Reconnecting to %s...", CONFIG_WIFI_SSID);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;

        default:
            ESP_LOGI(TAG, "[WIFI] Event base %s with ID %d", base, id);
            break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t base, int32_t id, void *event_data) {
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "[IP] Got IP:"
                IPSTR, IP2STR(&event->ip_info.ip));
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

void app_main() {
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_serial_ensure(&esp_serial));
    ESP_ERROR_CHECK(esp_device_init(&esp_device));
    ESP_ERROR_CHECK(esp_discovery_init(&esp_discovery));

    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_red_channel);
    ledc_channel_config(&ledc_green_channel);
    ledc_channel_config(&ledc_blue_channel);

#ifdef LED_HAS_WHITE

    ledc_channel_config(&ledc_n_white_channel);
    ledc_channel_config(&ledc_w_white_channel);

#endif

    ledc_fade_func_install(0);

    esp_err_t error = load_env_state(&env_state);

    if (error != ESP_OK) {
        ESP_LOGI(TAG, "[APP] Load state error %d", error);
    }

    apply_env_state(&env_state);

    char *printed_state = serialize_env_state(&env_state);
    ESP_LOGI(TAG, "[APP] Current state: %s", printed_state);

    wifi_init_sta();
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    mqtt_app_start();

    xTaskCreate(task_env_publish, "task_env_publish", 2048, NULL, 0, NULL);
    xTaskCreate(task_env_persist, "task_env_persist", 2048, NULL, 0, NULL);
    xTaskCreate(task_esp_publish, "task_esp_publish", 2048, NULL, 0, NULL);
    xTaskCreate(task_discovery_publish, "task_discovery_publish", 2048, NULL, 0, NULL);
}
