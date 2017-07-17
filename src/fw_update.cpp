#include "fw_update.h"

#include <Arduino.h>
#include <ESP8266httpUpdate.h>

static unsigned long const TIME_MS_HTTP_TIMEOUT = 5000;
static char const* UPDATE_HOST = "rasppi3";
static uint16_t const UPDATE_PORT = 80;
static char const* UPDATE_URL_VERSION = "/anemometer/version";
static char const* UPDATE_URL_FIRMWARE = "/anemometer/firmware.bin";

void checkForUpdates()
{
  Serial.print("Checking for updates ... ");
  Serial.flush();
  WiFiClient client;
  if (!client.connect(UPDATE_HOST, UPDATE_PORT))
  {
#if 0
    return RC_ERROR_HTTP_CONNECT;
#else
    return;
#endif
  }
  client.print("GET ");
  client.print(UPDATE_URL_VERSION);
  client.print(" HTTP/1.1\r\n");
  client.print("Host: ");
  client.print(UPDATE_HOST);
  client.print("\r\n");
  client.print("Connection: close\r\n\r\n");
  client.flush();
  unsigned long const timeout = millis();
  while(client.available() == 0)
  {
      if (millis() - timeout > TIME_MS_HTTP_TIMEOUT)
      {
          Serial.println("timeout");
          client.stop();
#if 0
          return RC_ERROR_HTTP_TIMEOUT;
#else
          return;
#endif
      }
  }
  long availableFirmwareVersion = 0;
  while(client.available())
  {
      String line = client.readStringUntil('\n');
      if (line.startsWith("AnemometerVersion: "))
      {
          availableFirmwareVersion = line.substring(strlen("AnemometerVersion: ")).toInt();
          break;
      }
  }

  Serial.print("online version: ");
  Serial.print(availableFirmwareVersion);
  Serial.print(", running version: ");
  Serial.println(ANEMOMETER_VERSION);

  if (availableFirmwareVersion > ANEMOMETER_VERSION)
  {
    Serial.print("Updating system ... ");
    Serial.flush();
    ESPhttpUpdate.update(UPDATE_HOST, UPDATE_PORT, UPDATE_URL_FIRMWARE);
    Serial.println("done.");
  }

#if 0
  return RC_OK;
#else
  return;
#endif
}

