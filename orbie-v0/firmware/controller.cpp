/*
    ORBiE v0 Controller Implementation
    Copyright (c) 2025 Dylan Brenneis

    This file contains the controller class implementation for ORBiE v0.
*/

#include "controller.h"
#include "config.h"

// Define static member variables
volatile bool Controller::buttonStateChanged = false;
volatile unsigned long Controller::buttonStateChangeTime = 0;
volatile bool Controller::buttonInterruptState = false;

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
    
    // Initialize button debouncing variables
    lastButtonReading = false;
    debouncedButtonState = false;
    lastDebounceTime = 0;
    
    // Initialize button event detection variables
    lastButtonState = false;
    buttonPressInProgress = false;
    buttonPressStartTime = 0;
    doubleClickTimer = 0;
    doubleClickDetected = false;
    pendingReward = 0.0;
    
    // Initialize interrupt variables
    buttonStateChanged = false;
    buttonStateChangeTime = 0;
    buttonInterruptState = false;
}

// Hardware initialization
bool Controller::beginHardware() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);
    
    bool success = true;
    
    // Initialize button pin with interrupt
    pinMode(buttonPin, INPUT_PULLUP);
    
    // Attach interrupt for button state changes
    // This works on both Arduino Nano (pin 2) and Teensy 4.1
    attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, CHANGE);
    
    // Initialize WS2812B LED
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
    
    // Initialize servos
    rightServo.attach(rightServoPin);
    leftServo.attach(leftServoPin);
    setNeutralPosition();  
    
    // Initialize I2C for IMU
    Wire.begin();
    
    // Initialize BNO055 IMU
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055 IMU");
        success = false;
    } else {
        Serial.println("BNO055 IMU initialized successfully");
        
        // Wait for IMU to stabilize
        delay(1000);

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
    // This function provides debounced button reading
    // It can be used like a direct digitalRead() but with built-in debouncing
    // The function is non-blocking and returns the current debounced state
    
    bool currentReading = !digitalRead(buttonPin);  // Button is active low with pullup
    
    // Debounce logic
    if (currentReading != lastButtonReading) {
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (currentReading != debouncedButtonState) {
            debouncedButtonState = currentReading;
        }
    }
    
    lastButtonReading = currentReading;
    return debouncedButtonState;
}

bool Controller::wasButtonPressed() {
    bool currentState = isButtonPressed();
    bool wasPressed = !buttonPressed && currentState;  // Detect rising edge (button press)
    buttonPressed = currentState;
    return wasPressed;
}

// Interrupt Service Routine - called whenever button state changes
void Controller::buttonISR() {
    // Record the button state change immediately
    buttonStateChanged = true;
    buttonStateChangeTime = millis();
    buttonInterruptState = !digitalRead(BUTTON_PIN); // Button is active low
}

Controller::ButtonEvent Controller::checkButtonEvents() {
    ButtonEvent event = {0.0, false};  // Initialize with no reward and no query
    
    // Check if interrupt detected a state change
    if (buttonStateChanged) {
        bool currentState = buttonInterruptState;
        
        // Detect button press (rising edge)
        if (currentState && !lastButtonState) {
            // Check if this is a double-click
            if (doubleClickTimer > 0 && (buttonStateChangeTime - doubleClickTimer) < DOUBLE_CLICK_TIME) {
                // Second press detected - this is a double-click!
                event.query = true;
                event.reward = 0; // Override any previous reward
                doubleClickTimer = 0;
                buttonPressInProgress = false;
                pendingReward = 0.0; // Clear pending reward for double-click
                Serial.println("Double click detected - requesting new action");
            } else {
                // First press - start tracking
                buttonPressInProgress = true;
                buttonPressStartTime = buttonStateChangeTime;
            }
        }
        
        // Detect button release (falling edge)
        if (!currentState && lastButtonState && buttonPressInProgress) {
            // Button was released - calculate reward but don't assign it yet
            unsigned long pressDuration = buttonStateChangeTime - buttonPressStartTime;
            float calculatedReward = pressDuration * 0.001; // Convert ms to reward (1ms = 0.001 reward)
            
            // Start double-click detection timer
            doubleClickTimer = buttonStateChangeTime;
            buttonPressInProgress = false;
            
            // Store the calculated reward for potential assignment later
            pendingReward = calculatedReward;
        }
        
        // Update last state
        lastButtonState = currentState;
        
        // Clear the interrupt flag
        buttonStateChanged = false;
    }
    
    // Clear timer if window expired and assign pending reward
    if (doubleClickTimer > 0 && (millis() - doubleClickTimer) >= DOUBLE_CLICK_TIME) {
        // Double-click window expired - this was a single click
        if (pendingReward > 0) {
            event.reward = pendingReward;
            Serial.println("Single click - reward added");
        }
        doubleClickTimer = 0;
        pendingReward = 0.0;
    }
    
    return event;
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
    
    return true;
}

float Controller::getHeading() {
    return orientationData.orientation.x;
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

void Controller::setNeutralPosition() {
    setRightServo(170);
    setLeftServo(10);
    setLedColor(255, 255, 255);
}

void Controller::setLedColor(int r, int g, int b) {
    leds[0] = CRGB(r, g, b);
    FastLED.show();
}
