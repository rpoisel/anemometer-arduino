#include "config.h"

#include <FS.h>

static char const* KEY_JSON_CONFIG = "config";
static char const* KEY_JSON_WLAN = "wlan";
static char const* KEY_JSON_MQTT = "mqtt";

RC ConfigProviderCb::parseWifiSettings(ConfigCb cb)
{
  return parseSettings(KEY_JSON_WLAN, cb, RC_ERROR_WIFI_CONNECT);
}

RC ConfigProviderCb::parseMqttSettings(ConfigCb cb)
{
  return parseSettings(KEY_JSON_MQTT, cb, RC_ERROR_MQTT_CONNECT);
}

RC ConfigProviderCb::parseSettings(char const* key, ConfigCb cb, RC errorCode)
{
  File f = SPIFFS.open(path, "r");
  if (!f)
  {
    Serial.print("Could not open file '");
    Serial.print(path);
    Serial.println("'");
    return RC_ERROR_WIFI_CONFIG;
  }
  String config = f.readString();
  const size_t bufferSize = 2 * JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(3) + 190;
  DynamicJsonBuffer jsonBuffer(bufferSize);
  JsonObject& root = jsonBuffer.parseObject(config.c_str());
  JsonObject& config_object = root[KEY_JSON_CONFIG][key];
  for (auto config_pair : config_object)
  {
    if (cb(config_pair.key, config_pair.value))
    {
      return RC_OK;
    }
  }
  return errorCode;
}
