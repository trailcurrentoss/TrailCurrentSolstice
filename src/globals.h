#pragma once
#include <Arduino.h>
#include "driver/twai.h"

#define DEBUG 1 // This sets whether or not the Serial.println and Serial.print will even be compiled

// Waveshare ESP32-S3-RS485-CAN pin assignments
#define VICTRON_RX_PIN 1   // GPIO1 - Serial RX from Victron VE.Direct
#define VICTRON_TX_PIN 2   // GPIO2 - Serial TX to Victron VE.Direct (for HEX commands)

// Conditional definition for debugging if DEBUG is 1 then it will print to serial port.
// If DEBUG = 0 then the lines will be removed by the compiler.
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(x,y) Serial.printf(x,y)
#define debugg(x,y,z) Serial.printf(x,y,z)
#else
#define debug(x)
#define debugln(x)
#define debugf(x,y)
#define debugg(x,y,z)
#endif

namespace globals {

}
