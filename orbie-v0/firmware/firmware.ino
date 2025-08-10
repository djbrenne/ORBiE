/*
    ORBiE v0 Firmware
    Copyright (c) 2025 Dylan Brenneis

    This file contains the main entry point for the ORBiE v0 firmware.
*/

#include "config.h"
#include "controller.h"

// Create controller instance
Controller controller;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    controller.beginHardware();
}

void loop() {
    // Update controller state (IMU data, button state)
    controller.update();
    
    // Print magnetometer data
    Serial.print("Magnetometer: ");
    Serial.print(controller.getHeading());
    Serial.print(" ");
    Serial.print(controller.getPitch());
    Serial.print(" ");
    Serial.println(controller.getRoll());
    
    // Check for button press
    if (controller.wasButtonPressed()) {
        Serial.println("*** BUTTON PRESSED! ***");
    }
    
    // Flash the LED
    controller.toggleLed();
    delay(300);  // Small delay to prevent overwhelming the system
}
