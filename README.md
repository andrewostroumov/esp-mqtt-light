# Home Assistant ESP32 MQTT Light Adapter

This will help you to build Home Assistant MQTT adapter to control any kind of light

Has build with esp-idf commit 93a8603c545fb8e54741d6685146e2f3b874378d

### Hardware Required

This firmware can be executed on any ESP32 board, the only required interface is WiFi and connection to internet.

### Configure the project

You have to config wifi SSID / PASSWORD and MQTT broker URI:

```shell script
idf.py menuconfig
```

Example of config:
```shell script
WIFI_SSID: Xiaomi
WIFI_SSID: password
MQTT_URI: mqtts://192.168.31.116:8883
```

### Security

You have to provide server CA certificate in the security folder in PEM format

```shell script
cat /mosquitto/security/ca.crt > security/ca.crt
```

### Build and Flash
Build the project and flash it to the board, then run monitor tool to view serial output:

```shell script
idf.py -p PORT flash monitor
```
(To exit the serial monitor, type Ctrl-].)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Using

Now it's just read data from one MQTT topic update current state and write to another

```shell script
Read topic: discovery/light/esp32_yellow_garland/set
Write topic: discovery/light/esp32_yellow_garland/state
```

I know that I have to move it in the config system and provide defaults