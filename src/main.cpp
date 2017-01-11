#include "Arduino.h"
#include <Wire.h> // For I2C communication between various components

// 7 Segment Backpack libraries:
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"


#ifndef SYMBOL
#define expanderAddress 0x20 // i2c address of MCP23017
#define portADir 0x00 // Direction Register address for PORT A (pins 0-7 will be PORT A)
#define portBDir 0x01 // Direction Register address for PORT B (pins 8 - 15 will be PORT B)

// GPIO PORT A and B addressing
#define portA 0x12
#define portB 0x13

// RTC address
#define timeAddress 0x68 // I2C address to access time registers on the Chronodot
// BCD to decimal converter for values read from the RTC
#define bcdToDec(bcdVal) ((bcdVal / 16 * 10) + (bcdVal % 16)) // preprocessor definition of a function
// everytime the compiler encounters bcdToDec(input) it will expand to:
// ((bcdVal / 16 * 10) + (bcdVal % 16))
#endif

void mcPinMode(uint8_t pin, bool state) {
}

void mcWrite(uint8_t pin, bool state) {
}

// RTC time collection functions

uint8_t hour() { // returns hour (0 - 24)
    Wire.beginTransmission(timeAddress);
    Wire.write(0x02); // start reading data from hours register on the Chronodot
    Wire.endTransmission();
    // request hour data
    Wire.requestFrom(timeAddress, 2); // request one byte from the RTC since we start reading data from the hours register we will get hours
    Wire.read();
    uint8_t hours = bcdToDec(Wire.read() & 0x3f); // mask required due to control bits on hour register
    return hours;
}

uint8_t minute() { // returns minute (0 - 59)
    Wire.beginTransmission(timeAddress);
    Wire.write(0x01); // start at minutes register on Chronodot
    Wire.endTransmission();
    // request minute data
    Wire.requestFrom(timeAddress, 1); // request one byte which is the minute value
    uint8_t minutes = bcdToDec(Wire.read()); // no mask required on minute register simply convert to decimal
    return minutes;
}

void initIO() {
    Wire.begin();
    pinMode(A4, OUTPUT);
    pinMode(A5,  OUTPUT);
    Serial.begin(9600);
}

void setup() {
    initIO();
    Wire.beginTransmission(0x20);
    Wire.write(0x01); // IODIRB register
    Wire.write(0x00); // set all of port B to outputs
    Wire.endTransmission();
    Wire.beginTransmission(0x20);
    Wire.write(0x13); // GPIOB
    Wire.write(255); // port B LSB First
    Wire.endTransmission();
}


void loop() {
    Serial.println(hour());
    delay(1000);
}
