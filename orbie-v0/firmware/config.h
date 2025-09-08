/*
    This file contains the configuration for the firmware.
*/

// Setup Parameters
#define SERIAL_BAUD_RATE 115200

// Pin definitions
#define BUTTON_PIN 2
#define LED_PIN 9
#define R_SERVO_PIN 5
#define L_SERVO_PIN 6
#define BNO055_ADDRESS 0x55

// LED definitions
#define LED_COUNT 1
#define LED_BRIGHTNESS 255

// Servo definitions
#define R_SERVO_MIN_ANGLE 125
#define R_SERVO_MAX_ANGLE 25
#define L_SERVO_MIN_ANGLE 145
#define L_SERVO_MAX_ANGLE 50

// Debounce delay
#define DEBOUNCE_DELAY 20

// Button timing
#define DOUBLE_CLICK_TIME 300  // 300ms for double click detection (reduced from 500ms)

// Agent definitions
#define UNPROMPTED_ACTION_TIMEOUT_MIN 6000  // 1 minute minimum for unprompted action timeout
#define UNPROMPTED_ACTION_TIMEOUT_MAX 60000  // 10 minutes maximum for unprompted action timeout
