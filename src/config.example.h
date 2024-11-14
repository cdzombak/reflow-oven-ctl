#define CFG_HOSTNAME "reflow-oven"
#define CFG_WIFI_ESSID "mywifinet"
#define CFG_WIFI_PASSWORD "mywifipassword"

#define CFG_MQTT_HOST "mynas.lan"
#define CFG_MQTT_PORT 1883
#define CFG_MQTT_USER "reflow-oven"
#define CFG_MQTT_PASSWORD "mymqttpassword"

#define CFG_MQTT_COMMAND_TOPIC "reflow-oven/cmd/#" // must end with /#
#define CFG_MQTT_MESSAGE_TOPIC "reflow-oven/msg"
#define CFG_MQTT_STATE_TOPIC "reflow-oven/state"

// optional; if these aren't set, the relevant string operations and MQTT publishes will be skipped:
#define CFG_MQTT_DEBUG_TOPIC "reflow-oven/debug"
#define CFG_MQTT_OTA_TOPIC "reflow-oven/ota"
