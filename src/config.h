#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

#include <ArduinoJson.h>
#include <Arduino.h>

#include <functional>

typedef std::function<CF (String const& key, String const& value)> ConfigCb;

class ConfigProviderCb
{
public:
  ConfigProviderCb(char const* configPath) : path(configPath) {}
  RC parseWifiSettings(ConfigCb cb);
  RC parseMqttSettings(ConfigCb cb);
private:
  RC parseSettings(char const* key, ConfigCb cb, RC errorCode);
  char const* path;
};

#endif /* CONFIG_H */
