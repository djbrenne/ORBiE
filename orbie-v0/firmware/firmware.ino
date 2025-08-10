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
    controller.update();
}