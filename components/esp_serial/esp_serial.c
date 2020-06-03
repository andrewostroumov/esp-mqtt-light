#include <lwipopts.h>
#include "nvs_flash.h"
#include "esp_serial.h"

esp_err_t esp_serial_ensure(esp_serial_t *esp_serial) {
    esp_err_t err;
    nvs_handle_t nvs;

    err = nvs_open(esp_serial->namespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    size_t required_size = 0;
    err = nvs_get_blob(nvs, esp_serial->key, NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    if (required_size) {
        err = nvs_get_blob(nvs, esp_serial->key, esp_serial->data, &required_size);
        if (err != ESP_OK) return err;
    } else {
        esp_fill_random(esp_serial->data, esp_serial->length);
        err = nvs_set_blob(nvs, esp_serial->key, esp_serial->data, esp_serial->length);
        if (err != ESP_OK) return err;
    }

    nvs_close(nvs);
    return ESP_OK;
}

void esp_serial_hex(esp_serial_t *esp_serial, char **hex) {
    *hex = malloc(esp_serial->length * 2 + 1);
    char *p = *hex;

    for (size_t j = 0; j < esp_serial->length; ++j) {
        p += sprintf(p, "%02x", esp_serial->data[j]);
    }
}
