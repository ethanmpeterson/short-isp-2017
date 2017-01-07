#include "Arduino.h"
#include <Wire.h>

#ifndef SYMBOL
#define address 0x20 // i2c address of MCP23017
#define portADir 0x00 // Direction Register address for PORT A (pins 0-7 will be PORT A)
#define portBDir 0x01 // Direction Register address for PORT B (pins 8 - 15 will be PORT B)

// GPIO PORT A and B addressing
#define portA 0x12
#define portB 0x13
#endif

void setup() {
}


void loop() {
}
