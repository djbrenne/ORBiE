/*
    ORBiE v0 Controller Implementation
    Copyright (c) 2025 Dylan Brenneis

    This file contains the controller class implementation for ORBiE v0.
*/

#include "controller.h"

// Constructor
Controller::Controller() {
    // Initialize pin assignments from config
    buttonPin = BUTTON_PIN;
    ledPin = LED_PIN;
    rightServoPin = R_SERVO_PIN;
    leftServoPin = L_SERVO_PIN;
    
    // Initialize state variables
    buttonPressed = false;
    ledState = false;
    rightServoAngle = 90;  // Center position
    leftServoAngle = 90;   // Center position
}

// Hardware initialization
bool Controller::beginHardware() {
    bool success = true;
    
    // Initialize button pin
    pinMode(buttonPin, INPUT_PULLUP);
    
    // Initialize WS2812B LED
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
    
    // Initialize servos
    rightServo.attach(rightServoPin);
    leftServo.attach(leftServoPin);
    
    // Set servos to center position
    rightServo.write(rightServoAngle);
    leftServo.write(leftServoAngle);
    
    // Initialize I2C for IMU
    Wire.begin();
    
    // Initialize BNO055 IMU
    if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
        Serial.println("Failed to initialize BNO055 IMU");
        success = false;
    } else {
        Serial.println("BNO055 IMU initialized successfully");
        
        // Wait for IMU to stabilize
        delay(1000);
        
        // Set up IMU for absolute orientation
        bno.setExtCrystalUse(true);
    }
    
    if (success) {
        Serial.println("Hardware initialization completed successfully");
    } else {
        Serial.println("Hardware initialization failed");
        while (1) {
            delay(1000);  // Halt execution
            // TODO: Add LED indicator for failure
        }
    }
    
    return success;
}

// Button functions
bool Controller::isButtonPressed() {
    return !digitalRead(buttonPin);  // Button is active low with pullup
}

bool Controller::wasButtonPressed() {
    bool currentState = isButtonPressed();
    bool wasPressed = !buttonPressed && currentState;  // Detect rising edge (button press)
    buttonPressed = currentState;
    return wasPressed;
}

// LED functions
void Controller::setLed(bool state) {
    ledState = state;
    if (state) {
        leds[0] = CRGB::White;  // Turn on with white color
    } else {
        leds[0] = CRGB::Black;  // Turn off
    }
    FastLED.show();
}

void Controller::toggleLed() {
    setLed(!ledState);
}

bool Controller::getLedState() {
    return ledState;
}

// Servo functions
void Controller::setRightServo(int angle) {
    angle = constrain(angle, R_SERVO_MIN_ANGLE, R_SERVO_MAX_ANGLE);  // Limit servo range
    rightServoAngle = angle;
    rightServo.write(angle);
}

void Controller::setLeftServo(int angle) {
    angle = constrain(angle, L_SERVO_MIN_ANGLE, L_SERVO_MAX_ANGLE);  // Limit servo range
    leftServoAngle = angle;
    leftServo.write(angle);
}

int Controller::getRightServoAngle() {
    return rightServoAngle;
}

int Controller::getLeftServoAngle() {
    return leftServoAngle;
}

// IMU functions
bool Controller::updateImuData() {
    // Get orientation data
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    
    // Get angular velocity data
    bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    // Get linear acceleration data
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    return true;
}

float Controller::getHeading() {
    return orientationData.orientation.x;
}

float Controller::getPitch() {
    return orientationData.orientation.y;
}

float Controller::getRoll() {
    return orientationData.orientation.z;
}

float Controller::getAngularVelocityX() {
    return angVelData.gyro.x;
}

float Controller::getAngularVelocityY() {
    return angVelData.gyro.y;
}

float Controller::getAngularVelocityZ() {
    return angVelData.gyro.z;
}

float Controller::getLinearAccelX() {
    return linearAccelData.acceleration.x;
}

float Controller::getLinearAccelY() {
    return linearAccelData.acceleration.y;
}

float Controller::getLinearAccelZ() {
    return linearAccelData.acceleration.z;
}

// Utility functions
void Controller::update() {
    // Update IMU data
    updateImuData();
    
    // Update button state
    isButtonPressed();  // This updates the internal state
}

void Controller::reset() {
    // Reset servos to center
    setRightServo(90);
    setLeftServo(90);
    
    // Turn off LED
    setLed(false);
    
    // Reset state variables
    buttonPressed = false;
}