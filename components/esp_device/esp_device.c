#include "esp_system.h"
#include "esp_device.h"

#define DEVICE_MANUFACTURER "espressif"
#define DEVICE_MODEL "esp32"

esp_err_t esp_device_init(esp_device_t *device) {
    device->manufacturer = DEVICE_MANUFACTURER;
    device->model = DEVICE_MODEL;
    device->idf_version = (char *) esp_get_idf_version();

    return ESP_OK;
}
