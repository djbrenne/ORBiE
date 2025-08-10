/*
    ORBiE v0 Controller Header
    Copyright (c) 2025 Dylan Brenneis

    This file contains the controller class declarations for ORBiE v0.
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "config.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <FastLED.h>

class Controller {
private:
    // Hardware objects
    Servo rightServo;
    Servo leftServo;
    Adafruit_BNO055 bno;
    CRGB leds[LED_COUNT];
    
    // Pin assignments
    uint8_t buttonPin;
    uint8_t ledPin;
    uint8_t rightServoPin;
    uint8_t leftServoPin;
    
    // State variables
    bool buttonPressed;
    bool ledState;
    int rightServoAngle;
    int leftServoAngle;
    
    // IMU data
    sensors_event_t orientationData;
    sensors_event_t angVelData;
    sensors_event_t linearAccelData;

public:
    // Constructor
    Controller();
    
    // Hardware initialization
    bool beginHardware();
    
    // Button functions
    bool isButtonPressed();
    bool wasButtonPressed();
    
    // Servo functions
    void setRightServo(int angle);
    void setLeftServo(int angle);
    int getRightServoAngle();
    int getLeftServoAngle();
    
    // LED functions
    void setLed(bool state);
    void setLedColor(int r, int g, int b);
    bool getLedState();
    void toggleLed();

    // IMU functions
    bool updateImuData();
    float getHeading();
    float getPitch();
    float getRoll();
    float getAngularVelocityX();
    float getAngularVelocityY();
    float getAngularVelocityZ();
    float getLinearAccelX();
    float getLinearAccelY();
    float getLinearAccelZ();
    
    // Utility functions
    void update();
    void reset();
    void setNeutralPosition();
};

#endif // CONTROLLER_H
