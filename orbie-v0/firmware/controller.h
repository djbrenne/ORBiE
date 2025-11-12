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
    int potentiometerValue;
    
    // Pin assignments
    uint8_t buttonPin;
    uint8_t ledPin;
    uint8_t rightServoPin;
    uint8_t leftServoPin;
    uint8_t potentiometerPin;
    
    // State variables
    bool buttonPressed;
    bool ledState;
    int rightServoAngle;
    int leftServoAngle;
    
    // Button debouncing variables
    bool lastButtonReading;
    bool debouncedButtonState;
    unsigned long lastDebounceTime;
    
    // Button event detection variables
    bool lastButtonState;
    bool buttonPressInProgress;
    unsigned long buttonPressStartTime;
    unsigned long doubleClickTimer;
    bool doubleClickDetected;
    float pendingReward;
    // DOUBLE_CLICK_TIME is now defined in config.h
    
    // Interrupt-based button detection
    static volatile bool buttonStateChanged;
    static volatile unsigned long buttonStateChangeTime;
    static volatile bool buttonInterruptState;
    
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
    
    // Button event detection
    struct ButtonEvent {
        float reward;      // Reward accumulated from button press duration
        bool query;        // True if double-click detected (action request)
    };
    ButtonEvent checkButtonEvents();  // Returns both reward and action request status
    
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

    // Action functions
    void goForward();
    void turnLeft();
    void turnRight();
    
    // Potentiometer functions
    float getPotentiometerValue();
    unsigned long calculateUnpromptedActionTimeout();
    void getTimeoutBounds(float& L, float& U);  // Get calculated L and U bounds for testing
    
    // Interrupt service routine (must be static)
    static void buttonISR();
};

#endif // CONTROLLER_H
