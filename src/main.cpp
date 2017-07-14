#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <cstdio>

typedef uint8_t RC;
#define RC_OK                  0x00
#define RC_ERROR               0x80
#define RC_ERROR_WIFI_CONNECT  0x81
#define RC_ERROR_MQTT_CONNECT  0x82

#define RC_FAILED(x)           ((x) & RC_ERROR)
#define RC_SUCCEEDED(x)        (!RC_FAILED(x))

static RC initSystem();
static float measureWindSpeed();
static void evaluateWindSpeed(float windSpeed);
static RC connectToWifi();
static void publishMqttMessage();
static RC connectToMqttBroker();
static void deepSleep();
static void isrRotation();

static uint8_t const PIN_SUPPLY = D2;
static uint8_t const PIN_INTERRUPT = D1;
static unsigned long const TIME_MS_DEBOUNCE = 15;
static unsigned long const TIME_MS_WIFI_ESTABLISH_CONNECTION = 5000;
static unsigned long const TIME_MS_MEASURE_PERIOD = 3000;
static size_t const THRESHOLD_CNT_WIND_SPEED = 2;
static char const* MQTT_BROKER = "xxx";
static char const* MQTT_TOPIC = "windspeed";
static char const* WIFI_SSID = "yyy";
static char const* WIFI_PASS = "zzz";

static volatile size_t cntWindSpeed;
static volatile unsigned long ContactBounceTime;
static WiFiClient espClient;
static PubSubClient mqttClient(espClient);

void setup()
{
  if (RC_FAILED(initSystem()))
  {
    return;
  }

  float windSpeed = measureWindSpeed();
  evaluateWindSpeed(windSpeed);

  deepSleep();
}

void loop()
{
  /* empty on purpose */
}

static RC initSystem()
{
  Serial.begin(9600);
  pinMode(PIN_SUPPLY, OUTPUT);
  pinMode(PIN_INTERRUPT, INPUT_PULLUP);

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
  if (windspeed <= THRESHOLD_CNT_WIND_SPEED)
  {
    return;
  }
  if (RC_FAILED(connectToWifi()))
  {
    return;
  }
  publishMqttMessage();
}

static RC connectToWifi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // WiFi fix: https://github.com/esp8266/Arduino/issues/2186
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long wifiConnectStart = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    if (WiFi.status() == WL_CONNECT_FAILED)
    {
      Serial.println("Failed to connect to WiFi. Please verify credentials: ");
      return RC_ERROR_WIFI_CONNECT;
    }

    delay(500);
    Serial.println("...");
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

  if (RC_FAILED(connectToMqttBroker()))
  {
    return;
  }

  snprintf(mqttMessage, sizeof(mqttMessage), "%u", cntWindSpeed);
  Serial.println("Publishing MQTT message ...");
  mqttClient.publish(MQTT_TOPIC, mqttMessage);
}

static RC connectToMqttBroker()
{
  mqttClient.setServer(MQTT_BROKER, 1883);

  for (size_t cnt = 0; cnt < 5; cnt++)
  {
    Serial.println("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str()))
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
