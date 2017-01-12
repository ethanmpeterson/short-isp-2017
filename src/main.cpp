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

// 7-Segment Backpack Address:
#define displayAddress 0x70

// RTC address
#define timeAddress 0x68 // I2C address to access time registers on the Chronodot
// BCD to decimal converter for values read from the RTC
#define bcdToDec(bcdVal) (((bcdVal & 0b11110000) >> 4) * 10 + (bcdVal & 0b00001111)) // preprocessor definition of a function
// everytime the compiler encounters bcdToDec(input) it will expand to:
// ((bcdVal / 16 * 10) + (bcdVal % 16))

#define interval 1000 // interval of time between reads of the rtc
#endif

Adafruit_7segment clockDisplay = Adafruit_7segment(); // instanciate library class for the 7-Segment Backpack

// make variables to hold the values in each of the MCP23017's registers
// pin direction registers
uint8_t portADirValue = 255;
uint8_t portBDirValue = 255;
// pin value registers
uint8_t portAValue = 0;
uint8_t portBValue = 0;

// make variables to store minute and hour that will be updated every second
uint8_t minutes;
uint8_t hours;

//MCP23017 functions

void mcPinMode(uint8_t pin, bool state) {
    Wire.beginTransmission(expanderAddress);
    if (pin < 8) {
        Wire.write(portADir); // map pins 0-7 to register A
        portADirValue ^= (~(-state) ^ portADirValue) & (1 << pin); // change bit at pin location to whatever the state boolean is
        // state boolean is inverted because OUTPUT and INPUT values are inverted on MCP23017 (OUTPUT = 0, INPUT = 0)
        Wire.write(portADirValue);
    } else { // if is greater or equal to 8
        Wire.write(portBDir);
        portBDirValue ^= (~(-state) ^ portBDirValue) & (1 << (pin - 8));
        Wire.write(portBDirValue);
    }
    Wire.endTransmission();
}


void mcWrite(uint8_t pin, bool state) {
    Wire.beginTransmission(expanderAddress);
    if (pin < 8) {
        Wire.write(portA);
        portAValue ^= (-state ^ portAValue) & (1 << pin);
        Wire.write(portAValue);
    } else {
        Wire.write(portB);
        portBValue ^= (-state ^ portBValue) & (1 << (pin - 8));
        Wire.write(portBValue);
    }
    Wire.endTransmission();
}

void mcRead() { // function serving the same purpose as digitalRead() that works for the MCP23017
    //Wire.beginTransmission(expanderAddress);

}

// RTC time collection functions

uint8_t hour() { // returns hour (0 - 24)
    Wire.beginTransmission(timeAddress);
    Wire.write(0x02); // start reading data from hours register on the Chronodot
    Wire.endTransmission();
    // request hour data
    Wire.requestFrom(timeAddress, 1); // request one byte from the RTC since we start reading data from the hours register we will get hours
    uint8_t hours = Wire.read(); // mask required due to control bits on hour register & 0x3f
    hours = (((hours & 0b00100000) >> 5) * 20 + ((hours & 0b00010000) >> 4) * 10 + (hours & 0b00001111)); // different BCD to Decimal conversion requied due to control bits on hour register
    return hours;
}


uint8_t minute() { // returns minute (0 - 59)
    Wire.beginTransmission(timeAddress);
    Wire.write(0x01); // start at minutes register on Chronodot
    Wire.endTransmission();
    // request minute data
    Wire.requestFrom(timeAddress, 1); // request one byte which is the minute value
    uint8_t minutes = Wire.read();
    return bcdToDec(minutes); // no mask required on minute register simply convert to decimal
}

// 7-Segment display functions

void showTime() {
    // setup variable to print to 7-Segment displays
    int displayVal = hour() * 100 + minute();
    Serial.println(String(displayVal));
    delay(1000);
}

void animateBargraph() { // bargraph animation for testing purposes
    for (int i = 0; i < 10; i++) {
        mcWrite(i, HIGH);
        delay(100);
    }
    for (int j = 9; j >= 0; j--) {
        mcWrite(j, LOW);
        delay(100);
    }
}

void initIO() {
    Wire.begin();
    pinMode(A4, OUTPUT);
    pinMode(A5,  OUTPUT);
    Serial.begin(9600);
    // Set all pins on MCP23017 to input by default
    Wire.beginTransmission(expanderAddress);
    Wire.write(portADir);
    Wire.write(255);
    Wire.endTransmission();
    // PORT B
    Wire.beginTransmission(expanderAddress);
    Wire.write(portBDir);
    Wire.write(255);
    Wire.endTransmission();
    mcPinMode(9, OUTPUT);
    for (uint8_t i = 0; i < 10; i++) { // Set pins 0-9 to OUTPUT for led bargraph
        mcPinMode(i, OUTPUT);
        mcWrite(i, LOW);
    }
    //initialize the 7-Segment Display
    clockDisplay.begin(displayAddress);
}

void setup() {
    initIO();
}



void loop() {
    String theTime = String(hour()) + ":" + String(minute());
    Serial.println(theTime);
    delay(1000);
}
