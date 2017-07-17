#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

extern uint32_t const ANEMOMETER_VERSION;

typedef uint8_t RC;
#define RC_OK                  0x00
#define RC_ERROR               0x80
#define RC_ERROR_WIFI_CONNECT  0x81
#define RC_ERROR_MQTT_CONNECT  0x82
#define RC_ERROR_HTTP_CONNECT  0x83
#define RC_ERROR_HTTP_TIMEOUT  0x84
#define RC_ERROR_WIFI_CONFIG   0x85
#define RC_ERROR_INIT_SYSTEM   0x86

#define RC_FAILED(x)           ((x) & RC_ERROR)
#define RC_SUCCEEDED(x)        (!RC_FAILED(x))

typedef bool CF;
#define CF_CONTINUE            false
#define CF_FINISH              true

#endif /* COMMON_H */
