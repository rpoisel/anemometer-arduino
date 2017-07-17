#include "common.h"
#include "config.h"
#include "fw_update.h"

#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <FS.h>

#include <cstdio>
#include <string>

static RC initSystem();
static float measureWindSpeed();
static void evaluateWindSpeed(float windSpeed);
static RC connectToWifi();
static RC connectToWifiImpl(char const* ssid, char const* password);
static void publishMqttMessage();
static RC connectToMqttBroker();
static void deepSleep();
static void isrRotation();

static uint8_t const PIN_SUPPLY = 14; // GPIO14
static uint8_t const PIN_INTERRUPT = 12; // GPIO12
static unsigned long const TIME_MS_DEBOUNCE = 15;
static unsigned long const TIME_MS_WIFI_ESTABLISH_CONNECTION = 10000;
static unsigned long const TIME_MS_MEASURE_PERIOD = 3000;
static size_t const THRESHOLD_CNT_WIND_SPEED = 2;
static char const* PATH_CONFIG_JSON = "/config.json";
static char const* MQTT_TOPIC = "windspeed";

static volatile size_t cntWindSpeed;
static volatile unsigned long ContactBounceTime;
static WiFiClient espClient;
static PubSubClient mqttClient(espClient);
static ConfigProviderCb configCb(PATH_CONFIG_JSON);
static String mqttBroker;
static uint16_t mqttPort;
static String mqttClientId;
static uint8_t mqttConnectRetries = -1;

void setup()
{
  if (RC_FAILED(initSystem()))
  {
    return;
  }

  float windSpeed = measureWindSpeed();
  evaluateWindSpeed(windSpeed);

  checkForUpdates();
  deepSleep();
}

void loop()
{
  /* empty on purpose */
}

static RC initSystem()
{
  Serial.begin(9600);
  Serial.print("Setting up system ... ");
  pinMode(PIN_SUPPLY, OUTPUT);
  pinMode(PIN_INTERRUPT, INPUT_PULLUP);

  if (!SPIFFS.begin())
  {
    Serial.println("Failed to open SPIFFS.");
    return RC_ERROR_INIT_SYSTEM;
  }

  Serial.println("done.");
  return RC_OK;
}

static void deepSleep()
{
  ESP.deepSleep(5e6 /* us */);
}

static float measureWindSpeed()
{
  digitalWrite(PIN_SUPPLY, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), isrRotation, FALLING);
  delay(TIME_MS_MEASURE_PERIOD);
  detachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT));
  return static_cast<float>(cntWindSpeed) / (TIME_MS_MEASURE_PERIOD / 1000 /* ms per second */);
}

static void evaluateWindSpeed(float windspeed)
{
  Serial.print("Windspeed: ");
  Serial.println(windspeed);
  if (windspeed <= THRESHOLD_CNT_WIND_SPEED)
  {
    return;
  }
  if (RC_FAILED(connectToWifi()))
  {
    return;
  }
  if (RC_FAILED(connectToMqttBroker()))
  {
    return;
  }
  publishMqttMessage();
}

static RC connectToWifi()
{
  return configCb.parseWifiSettings([](String const& ssid, String const& pass) -> RC {
    return RC_SUCCEEDED(connectToWifiImpl(ssid.c_str(), pass.c_str())) ? CF_FINISH : CF_CONTINUE;
  });
}

static RC connectToWifiImpl(char const* ssid, char const* password)
{
  WiFi.begin(ssid, password);

  // WiFi fix: https://github.com/esp8266/Arduino/issues/2186
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long wifiConnectStart = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    if (WiFi.status() == WL_CONNECT_FAILED)
    {
      Serial.println("Failed to connect to WiFi. Please verify credentials: ");
      return RC_ERROR_WIFI_CONNECT;
    }

    delay(500);
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println(" ... ");
    if (millis() - wifiConnectStart > TIME_MS_WIFI_ESTABLISH_CONNECTION)
    {
      Serial.println("Failed to connect to WiFi");
      return RC_ERROR_WIFI_CONNECT;
    }
  }
  Serial.println("Connected to WiFi.");
  return RC_OK;
}

static void publishMqttMessage()
{
  char mqttMessage[80];

  snprintf(mqttMessage, sizeof(mqttMessage), "%u", cntWindSpeed);
  Serial.println("Publishing MQTT message ...");
  mqttClient.publish(MQTT_TOPIC, mqttMessage);
}

CF mqttConfigCb(String const& key, String const& value)
{
    if (key.equalsIgnoreCase("host")) {
        mqttBroker = value;
    }
    else if (key.equalsIgnoreCase("port")) {
        mqttPort = value.toInt();
    }
    else if (key.equalsIgnoreCase("clientId")) {
        mqttClientId = value;
    }
    else if (key.equalsIgnoreCase("retries")) {
        mqttConnectRetries = value.toInt();
    }

    if (mqttBroker.length() > 0 && mqttPort != 0 && mqttClientId.length() > 0 && mqttConnectRetries > 0) {
      return CF_FINISH;
    }
    return CF_CONTINUE;
}

static RC connectToMqttBroker()
{
  RC rc = configCb.parseMqttSettings(mqttConfigCb);
  if (RC_FAILED(rc)) {
    return rc;
  };

  mqttClient.setServer(mqttBroker.c_str(), mqttPort);

  for (size_t cnt = 0; cnt < mqttConnectRetries; cnt++)
  {
    Serial.print("Attempting MQTT connection to ");
    Serial.print(mqttBroker);
    Serial.print(":");
    Serial.print(mqttPort);
    Serial.println(" ...");
    if (mqttClient.connect(mqttClientId.c_str()))
    {
      return RC_OK;
    }
    if (mqttClient.state() > 0)
    {
      return RC_ERROR_MQTT_CONNECT;
    }
  }
  return RC_ERROR_MQTT_CONNECT;
}

static void isrRotation()
{
  if (millis() - ContactBounceTime <= TIME_MS_DEBOUNCE)
  {
    return;
  }
  ContactBounceTime = millis();
  cntWindSpeed++;
}
