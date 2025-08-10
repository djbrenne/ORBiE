/*
    ORBiE v0 Firmware Action Animations
    Copyright (c) 2025 Dylan Brenneis

    This file contains the implementation of the action animations for the ORBiE v0 firmware.
*/

#include "action-animations.h"
#include <Arduino.h>

// Animation constants
const int ARM_DOWN_POS = 10;   // 10 degrees away from 0 (pointing down)
const int ARM_UP_POS = 170;    // 10 degrees away from 180 (pointing up)
const int ARM_MID_POS = 90;    // Arms held out to the side
const int MOVE_SPEED_DELAY = 15; // 15ms delay between each servo position update

// Global state for happy animation
bool happyAnimationActive = false;

// INTERNAL HELPER FUNCTIONS
void moveArm(Controller& controller, bool isLeftArm, int targetPos) {
    int currentPos = isLeftArm ? controller.getLeftServoAngle() : controller.getRightServoAngle();
    
    if (currentPos < targetPos) {
        for (int pos = currentPos; pos <= targetPos; pos++) {
            if (isLeftArm) {
                controller.setLeftServo(pos);
            } else {
                controller.setRightServo(pos);
            }
            delay(MOVE_SPEED_DELAY);
        }
    } else {
        for (int pos = currentPos; pos >= targetPos; pos--) {
            if (isLeftArm) {
                controller.setLeftServo(pos);
            } else {
                controller.setRightServo(pos);
            }
            delay(MOVE_SPEED_DELAY);
        }
    }
}

void moveBothArms(Controller& controller, int leftTarget, int rightTarget) {
    moveArm(controller, true, leftTarget);   // Left arm
    moveArm(controller, false, rightTarget); // Right arm
}

// PUBLIC FUNCTIONS
void executeActionAnimation(Controller& controller, int action) {
    happyAnimationActive = false;

    switch (action) {
        case ACTION_FORWARD: // Both arms up
            moveBothArms(controller, ARM_UP_POS, ARM_UP_POS);
            break;

        case ACTION_LEFT: // Left arm up, right arm down
            moveBothArms(controller, ARM_UP_POS, ARM_DOWN_POS);
            break;

        case ACTION_RIGHT: // Right arm up, left arm down
            moveBothArms(controller, ARM_DOWN_POS, ARM_UP_POS);
            break;

        case ACTION_AGITATED: // "Agitated" or "Needs Feedback" animation
            for (int i = 0; i < 3; i++) {
                moveBothArms(controller, ARM_MID_POS, ARM_MID_POS);
                delay(100);
                moveBothArms(controller, ARM_DOWN_POS, ARM_DOWN_POS);
                delay(100);
            }
            break;
        
        case ACTION_HAPPY_START:
            happyAnimationActive = true; // Enable the happy animation flag
            break;

        case ACTION_HAPPY_STOP:
            happyAnimationActive = false; // Disable the flag
            moveBothArms(controller, ARM_DOWN_POS, ARM_DOWN_POS);
            break;

        case ACTION_ARMS_DOWN:
            moveBothArms(controller, ARM_DOWN_POS, ARM_DOWN_POS);
            break;
    }
}

void updateHappyAnimation(Controller& controller) {
    // If the happy animation is not active, do nothing
    if (!happyAnimationActive) {
        return;
    }

    // Move arms in sine wave opposite to each other
    float time = millis() / 1000.0;
    float waveSpeed = 4.0;
    float waveAmplitude = (ARM_UP_POS - ARM_DOWN_POS) / 2.0;
    float waveOffset = ARM_DOWN_POS + waveAmplitude;

    int leftPos = waveOffset + waveAmplitude * sin(waveSpeed * time);
    int rightPos = waveOffset + waveAmplitude * sin(waveSpeed * time + PI); 

    controller.setLeftServo(leftPos);
    controller.setRightServo(rightPos);
}
