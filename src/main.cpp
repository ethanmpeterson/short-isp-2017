#include "Arduino.h"
#include <Wire.h> // For I2C communication between various components
#include <SoftwareSerial.h> // For Serial communication with the fingerprint sensor
#include <Servo.h> // For easy control of a servo for locking and unlocking the door

// 7 Segment Backpack libraries:
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

// Fingerprint Scanner Library:
#include <Adafruit_Fingerprint.h>

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

// define button pin
#define button 13
// define function to check if nth bit is set or cleared in byte
#define bitCheck(inputVar, position) ((inputVar) & (1 << position))
#define fingerId 0 // id # of my fingerprint stored in the senor
#define servoPin 9 // PWM pin on arduino used to control the Servo Motor
#define lockedPos 85 // servo position for locked door
#define openPos 180 // servo positon for unlocked door
#endif
// servo.write(180);
// delay(1000);
// servo.write(85);
// delay(1000);
Adafruit_7segment clockDisplay = Adafruit_7segment(); // instanciate library class for the 7-Segment Backpack

// intanciate fingerprint and software serial libraries:
SoftwareSerial mySerial(2, 3);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// create instance of Servo Library class
Servo servo;

// make variables to hold the values in each of the MCP23017's registers
// pin direction registers
uint8_t portADirValue = 255;
uint8_t portBDirValue = 255;
// pin value registers
uint8_t portAValue = 0;
uint8_t portBValue = 0;

bool buttonState;
bool locked;
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

bool mcRead(uint8_t pin) { // function serving the same purpose as digitalRead() that works for the MCP23017
    Wire.beginTransmission(expanderAddress);
    uint8_t bitToAccess = pin;
    uint8_t dataRead; // stores input byte obtained from MCP23017
    if (pin < 8) {
        Wire.write(portA);
    } else {
        Wire.write(portB);
        bitToAccess = pin - 8;
    }
    Wire.endTransmission();
    // request the data
    Wire.requestFrom(expanderAddress, 1);
    dataRead = Wire.read();
    return bitCheck(dataRead, bitToAccess); // returns true if the bit is set and false if not
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

// 7-Segment display functions (and other code to do with visual output)

void showTime() {
    // setup variable to print to 7-Segment displays
    uint8_t h = hour();
    uint8_t m = minute();
    int theTime = h * 100 + m;
    // do 24 to 12 hour time conversion
    if (h > 12) {
        theTime -= 1200;
    } else if (h == 0) { // handle midnight where h = 0
        theTime += 1200;
    }
    clockDisplay.print(theTime);
    clockDisplay.drawColon(true); // show colon on the display between hour and min values
    clockDisplay.writeDisplay(); // show changes on the display
}

void animateBargraph() { // bargraph animation for testing purposes
}

void showBars(uint8_t barNum) { // shows progress of door being unlocked on bargraph
    // clear Bargraph first
    for (int j = 9; j >= 1; j--) {
        mcWrite(j, LOW);
    }
    // display specified # of bars on led bargraph
    for (int i = 0; i < barNum; i++) {
        mcWrite(i, HIGH);
    }
}


typedef struct { // will be used to store pin numbers for the RGB LED pin of each color (right out of in class lesson)
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} RGBLed;
// assign pin values
RGBLed rgb = {10, 11, 12};


// Fingerprint sensor code
uint8_t getID() {
    uint8_t f = finger.getImage();
    if (f != FINGERPRINT_OK) {
        return -1;
    }

    f = finger.image2Tz();
    if (f != FINGERPRINT_OK) {
        return -1;
    }

    f = finger.fingerFastSearch();
    if (f != FINGERPRINT_OK) {
        return -1;
    }
    return finger.fingerID;
}

void lockState(bool state) { // changes door from locked to unlocked depending on bool passed to the function
    servo.attach(9);
    if (state) {
        for (int i = openPos; i >= lockedPos; i--) {
            servo.write(i);
            uint8_t bars = map(servo.read(), lockedPos, openPos, 0, 10);
            showBars(bars);
            delay(25);
        }
    } else {
        for (int j = lockedPos; j < openPos; j++) {
            servo.write(j);
            uint8_t bars = map(servo.read(), lockedPos, openPos, 0, 10);
            showBars(bars);
            delay(25);
        }
    }
    servo.detach(); // prevent noise
}

void scan() { // runs in a loop scanning for a correct fingerprint on the sensor
    bool fingerAvailable = finger.verifyPassword();
    if (getID() == fingerId && fingerAvailable) {
        mcWrite(rgb.green, HIGH);
        mcWrite(rgb.red, LOW);
        Serial.println("FOUND ID: ");
        Serial.print(fingerId);
        locked = !locked;
        lockState(locked);
    } else {
        mcWrite(rgb.green, LOW);
        mcWrite(rgb.red, HIGH);
    }
}

void buttonScan() {
    buttonState = mcRead(button);
    if (buttonState != mcRead(button)) {
        
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
    for (uint8_t i = 0; i < 10; i++) { // Set pins 0-9 to OUTPUT for led bargraph
        mcPinMode(i, OUTPUT);
        mcWrite(i, LOW);
    }
    //initialize the 7-Segment Display
    clockDisplay.begin(displayAddress);
    finger.begin(57600);
    // initialize Servo
    servo.attach(servoPin);
    // Set RGB LED pins to OUTPUT
    mcPinMode(rgb.red, OUTPUT);
    mcPinMode(rgb.green, OUTPUT);
    mcPinMode(rgb.blue, OUTPUT);
}


void setup() {
    initIO();
    delay(1000);
}


void loop() {
    scan();
    showTime();
}
