#include <string.h>
#include "cJSON.h"
#include "esp_discovery.h"

static const char *DISCOVERY_TAG = "homeassistant/light";
static const char *DEVICE_TAG = "zigbee";

static const char *DISCOVERY_PATH = "config";
static const char *SET_PATH = "set";
static const char *STATE_PATH = "state";
static const char *STATUS_PATH = "status";
static const char *ATTRS_PATH = "attributes";

char *esp_discovery_serialize(esp_discovery_t *discovery) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();
//    cJSON *device = cJSON_CreateObject();
//
//    if (discovery->esp_device) {
//        cJSON_AddStringToObject(device, "manufacturer", discovery->esp_device->manufacturer);
//        cJSON_AddStringToObject(device, "model", discovery->esp_device->model);
//        cJSON_AddStringToObject(device, "sw_version", discovery->esp_device->sw_version);
//        cJSON_AddStringToObject(device, "idf_version", discovery->esp_device->idf_version);
//
//        cJSON_AddItemToObject(root, "device", device);
//    }

    cJSON_AddStringToObject(root, "platform", discovery->platform);
    cJSON_AddStringToObject(root, "schema", discovery->schema);
    cJSON_AddStringToObject(root, "name", discovery->name);
    cJSON_AddStringToObject(root, "unique_id", discovery->unique_id);

    cJSON_AddStringToObject(root, "state_topic", discovery->state_topic);
    cJSON_AddStringToObject(root, "command_topic", discovery->set_topic);
    cJSON_AddStringToObject(root, "availability_topic", discovery->status_topic);
    cJSON_AddStringToObject(root, "json_attributes_topic", discovery->attributes_topic);

    cJSON_AddNumberToObject(root, "min_mireds", discovery->min_mireds);
    cJSON_AddNumberToObject(root, "max_mireds", discovery->max_mireds);

    cJSON_AddBoolToObject(root, "brightness", discovery->brightness);
    cJSON_AddBoolToObject(root, "color_temp", discovery->color_temp);
    cJSON_AddBoolToObject(root, "white_value", discovery->white_value);
    cJSON_AddBoolToObject(root, "rgb", discovery->rgb);
    cJSON_AddBoolToObject(root, "effect", discovery->effect);
    cJSON_AddBoolToObject(root, "retain", discovery->retain);
    cJSON_AddBoolToObject(root, "optimistic", discovery->optimistic);

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

esp_err_t esp_discovery_init(esp_discovery_t *discovery) {
    esp_discovery_ensure(discovery);
    return ESP_OK;
}

esp_err_t esp_discovery_ensure(esp_discovery_t *discovery) {
    char *hex;

    if (!discovery->esp_serial) {
        return ESP_FAIL;
    }

    esp_serial_hex(discovery->esp_serial, &hex);

    discovery->name = malloc(strlen(hex) + 2 + 1);
    discovery->unique_id = malloc(strlen(hex) + strlen(DEVICE_TAG) + 3 + 1);
    discovery->discovery_topic = malloc(strlen(DISCOVERY_TAG) + strlen(hex) + strlen(DISCOVERY_PATH) + 4 + 1);
    discovery->state_topic = malloc(strlen(DEVICE_TAG) + strlen(hex) + strlen(STATE_PATH) + 4 + 1);
    discovery->set_topic = malloc(strlen(DEVICE_TAG) + strlen(hex) + strlen(SET_PATH) + 4 + 1);
    discovery->status_topic = malloc(strlen(DEVICE_TAG) + strlen(hex) + strlen(STATUS_PATH) + 4 + 1);
    discovery->attributes_topic = malloc(strlen(DEVICE_TAG) + strlen(hex) + strlen(ATTRS_PATH) + 4 + 1);

    sprintf(discovery->name, "0x%s", hex);
    sprintf(discovery->unique_id, "0x%s_%s", hex, DEVICE_TAG);
    sprintf(discovery->discovery_topic, "%s/0x%s/%s", DISCOVERY_TAG, hex, DISCOVERY_PATH);
    sprintf(discovery->state_topic, "%s/0x%s/%s", DEVICE_TAG, hex, STATE_PATH);
    sprintf(discovery->set_topic, "%s/0x%s/%s", DEVICE_TAG, hex, SET_PATH);
    sprintf(discovery->status_topic, "%s/0x%s/%s", DEVICE_TAG, hex, STATUS_PATH);
    sprintf(discovery->attributes_topic, "%s/0x%s/%s", DEVICE_TAG, hex, ATTRS_PATH);

    free(hex);
    return ESP_OK;
}
