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
#define POTENTIOMETER_PIN A0

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

// Agent definitions (values in seconds)
#define UNPROMPTED_ACTION_TIMEOUT_MIN 1.0  // 1 second minimum for unprompted action timeout (lower bound at pot=0)
#define UNPROMPTED_ACTION_TIMEOUT_MAX 3600.0 // 1 hour maximum for unprompted action timeout (upper bound at pot=1)
#define UNPROMPTED_ACTION_TIMEOUT_L_MAX 1800.0  // 15 minutes - maximum for lower bound (at pot=1)
#define UNPROMPTED_ACTION_TIMEOUT_U_MIN 5.0  // 5 seconds - minimum for upper bound (at pot=0)
