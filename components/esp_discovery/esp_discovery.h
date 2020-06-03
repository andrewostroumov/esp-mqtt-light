#include <esp_err.h>
#include <stdbool.h>
#include "esp_device.h"
#include "esp_serial.h"

typedef struct {
    esp_serial_t *esp_serial;
    esp_device_t *esp_device;
    char *platform;
    char *schema;
    char *name;
    char *unique_id;
    char *discovery_topic;
    char *state_topic;
    char *set_topic;
    char *status_topic;
    char *attributes_topic;
    int min_mireds;
    int max_mireds;
    bool brightness;
    bool color_temp;
    bool white_value;
    bool rgb;
    bool effect;
    bool retain;
    bool optimistic;
} esp_discovery_t;

char *esp_discovery_serialize(esp_discovery_t *discovery);

esp_err_t esp_discovery_init(esp_discovery_t *discovery);

esp_err_t esp_discovery_ensure(esp_discovery_t *discovery);