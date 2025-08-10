/*
    ORBiE v0 Firmware Action Animations
    Copyright (c) 2025 Dylan Brenneis

    This file contains the implementation of the action animations for the ORBiE v0 firmware.
*/

#include "action-animations.h"
#include <Arduino.h>
#include <Servo.h>

// SERVO INITIALIZATION
const int LEFT_ARM_PIN = 5;  // Digital PWM pin for the left arm servo
const int RIGHT_ARM_PIN = 6; // Digital PWM pin for the right arm servo
const int ARM_DOWN_POS = 10;   // 10 degrees away from 0 (pointing down)
const int ARM_UP_POS = 170;    // 10 degrees away from 180 (pointing up)
const int ARM_MID_POS = 90;    // Arms held out to the side
const int MOVE_SPEED_DELAY = 15; // 15ms delay between each servo position update
Servo leftServo;
Servo rightServo;
bool happyAnimationActive = false;

// INTERNAL HELPER FUNCTIONS
void setupArms() {
    leftServo.attach(LEFT_ARM_PIN);
    rightServo.attach(RIGHT_ARM_PIN);
    leftServo.write(ARM_DOWN_POS);
    rightServo.write(ARM_DOWN_POS);
    delay(500); // Give the servos time to move to the initial position
}

void moveArm(Servo &arm, int targetPos) {
    int currentPos = arm.read();
    if (currentPos < targetPos) {
        for (int pos = currentPos; pos <= targetPos; pos++) {
            arm.write(pos);
            delay(MOVE_SPEED_DELAY); // Uses the global speed variable
        }
    } else {
        for (int pos = currentPos; pos >= targetPos; pos--) {
            arm.write(pos);
            delay(MOVE_SPEED_DELAY); // Uses the global speed variable
        }
    }
}

void moveBothArms(int leftTarget, int rightTarget) {
    moveArm(leftArmServo, leftTarget);
    moveArm(rightArmServo, rightTarget);
}

// PUBLIC FUNCTIONS
void setupArms() {
    leftArmServo.attach(LEFT_ARM_PIN);
    rightArmServo.attach(RIGHT_ARM_PIN);
    leftArmServo.write(ARM_DOWN_POS);
    rightArmServo.write(ARM_DOWN_POS);
    delay(500);
}

void executeActionAnimation(int action) {
    happyAnimationActive = false;

    switch (action) {
        case ACTION_FORWARD: // Both arms up
            moveBothArms(ARM_UP_POS, ARM_UP_POS);
            break;

        case ACTION_LEFT: // Left arm up, right arm down
            moveBothArms(ARM_UP_POS, ARM_DOWN_POS);
            break;

        case ACTION_RIGHT: // Right arm up, left arm down
            moveBothArms(ARM_DOWN_POS, ARM_UP_POS);
            break;

        case ACTION_AGITATED: // "Agitated" or "Needs Feedback" animation
            for (int i = 0; i < 3; i++) {
                moveBothArms(ARM_MID_POS, ARM_MID_POS);
                delay(100);
                moveBothArms(ARM_DOWN_POS, ARM_DOWN_POS);
                delay(100);
            }
            break;
        
        case ACTION_HAPPY_START:
            happyAnimationActive = true; // Enable the happy animation flag
            break;

        case ACTION_HAPPY_STOP:
            happyAnimationActive = false; // Disable the flag
            moveBothArms(ARM_DOWN_POS, ARM_DOWN_POS);
            break;

        case ACTION_ARMS_DOWN:
            moveBothArms(ARM_DOWN_POS, ARM_DOWN_POS);
            break;
    }
}

void updateHappyAnimation() {
    if (!happyAnimationActive) {
        return; // Do nothing if the animation isn't supposed to be active
    }

    // This animation uses a sine wave to create a smooth, continuous "wave" or "dance".
    float time = millis() / 1000.0; // Get current time in seconds
    float waveSpeed = 4.0; // Controls how fast the wave motion is. Higher is faster.
    float waveAmplitude = (ARM_UP_POS - ARM_DOWN_POS) / 2.0;
    float waveOffset = ARM_DOWN_POS + waveAmplitude;

    // Calculate the position for the left arm based on the sine wave
    int leftPos = waveOffset + waveAmplitude * sin(waveSpeed * time);
    // Calculate the right arm's position, making it move opposite to the left arm (phase-shifted by PI)
    int rightPos = waveOffset + waveAmplitude * sin(waveSpeed * time + PI); 

    // Write the calculated positions to the servos
    leftArmServo.write(leftPos);
    rightArmServo.write(rightPos);
}
