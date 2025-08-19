/*
  Servo Limit Setter for ORBiE v0
  Simple calibration tool to test servo positions
  
  Commands:
  - "L90" = Set left servo to 90 degrees
  - "R45" = Set right servo to 45 degrees
  - "LR90" = Set both servos to 90 degrees
  - "min" = Set both servos to min angle
  - "max" = Set both servos to max angle
  - "center" = Set both servos to 90 degrees
  - "sweep" = Sweep both servos through full range
*/

#include <Servo.h>

// Import pin definitions from firmware config
#define R_SERVO_PIN 5
#define L_SERVO_PIN 6
#define SERIAL_BAUD_RATE 115200

// Servo definitions (copy from config.h)
#define R_SERVO_MIN_ANGLE 170
#define R_SERVO_MAX_ANGLE 10
#define L_SERVO_MIN_ANGLE 170
#define L_SERVO_MAX_ANGLE 10

Servo leftServo;
Servo rightServo;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  
  // Attach servos
  leftServo.attach(L_SERVO_PIN);
  rightServo.attach(R_SERVO_PIN);
  
  // Set to center position
  leftServo.write(90);
  rightServo.write(90);
  
  Serial.println("ORBiE v0 Servo Calibration Tool");
  Serial.println("Commands:");
  Serial.println("  L<angle>  - Set left servo (e.g., L90)");
  Serial.println("  R<angle>  - Set right servo (e.g., R45)");
  Serial.println("  LR<angle> - Set both servos (e.g., LR90)");
  Serial.println("  min       - Set both to min angle");
  Serial.println("  max       - Set both to max angle");
  Serial.println("  center    - Set both to 90 degrees");
  Serial.println("  sweep     - Sweep through full range");
  Serial.println("  status    - Show current positions");
  Serial.println();
  Serial.print("Left servo pin: "); Serial.println(L_SERVO_PIN);
  Serial.print("Right servo pin: "); Serial.println(R_SERVO_PIN);
  Serial.print("Left range: "); Serial.print(L_SERVO_MIN_ANGLE); Serial.print("-"); Serial.println(L_SERVO_MAX_ANGLE);
  Serial.print("Right range: "); Serial.print(R_SERVO_MIN_ANGLE); Serial.print("-"); Serial.println(R_SERVO_MAX_ANGLE);
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command.startsWith("L") && command.length() > 1) {
      // Left servo command
      if (command.startsWith("LR")) {
        // Both servos
        int angle = command.substring(2).toInt();
        if (angle >= 0 && angle <= 180) {
          leftServo.write(angle);
          rightServo.write(angle);
          Serial.print("Set both servos to: "); Serial.println(angle);
        } else {
          Serial.println("Invalid angle (0-180)");
        }
      } else {
        // Left servo only
        int angle = command.substring(1).toInt();
        if (angle >= 0 && angle <= 180) {
          leftServo.write(angle);
          Serial.print("Set left servo to: "); Serial.println(angle);
        } else {
          Serial.println("Invalid angle (0-180)");
        }
      }
    } else if (command.startsWith("R") && command.length() > 1) {
      // Right servo command
      int angle = command.substring(1).toInt();
      if (angle >= 0 && angle <= 180) {
        rightServo.write(angle);
        Serial.print("Set right servo to: "); Serial.println(angle);
      } else {
        Serial.println("Invalid angle (0-180)");
      }
    } else if (command == "MIN") {
      leftServo.write(L_SERVO_MIN_ANGLE);
      rightServo.write(R_SERVO_MIN_ANGLE);
      Serial.print("Set both to min: L="); Serial.print(L_SERVO_MIN_ANGLE); 
      Serial.print(" R="); Serial.println(R_SERVO_MIN_ANGLE);
    } else if (command == "MAX") {
      leftServo.write(L_SERVO_MAX_ANGLE);
      rightServo.write(R_SERVO_MAX_ANGLE);
      Serial.print("Set both to max: L="); Serial.print(L_SERVO_MAX_ANGLE); 
      Serial.print(" R="); Serial.println(R_SERVO_MAX_ANGLE);
    } else if (command == "CENTER") {
      leftServo.write(90);
      rightServo.write(90);
      Serial.println("Set both to center (90 degrees)");
    } else if (command == "SWEEP") {
      Serial.println("Sweeping servos through range...");
      sweepServos();
    } else if (command == "STATUS") {
      Serial.println("Current servo positions:");
      Serial.print("Left: "); Serial.print(leftServo.read()); Serial.println(" degrees");
      Serial.print("Right: "); Serial.print(rightServo.read()); Serial.println(" degrees");
    } else {
      Serial.println("Unknown command. Type 'help' for commands.");
    }
  }
}

void sweepServos() {
  // Sweep from min to max
  for (int angle = 0; angle <= 180; angle += 5) {
    leftServo.write(angle);
    rightServo.write(angle);
    Serial.print("Sweeping to: "); Serial.println(angle);
    delay(100);
  }
  
  // Sweep back
  for (int angle = 180; angle >= 0; angle -= 5) {
    leftServo.write(angle);
    rightServo.write(angle);
    Serial.print("Sweeping to: "); Serial.println(angle);
    delay(100);
  }
  
  // Return to center
  leftServo.write(90);
  rightServo.write(90);
  Serial.println("Sweep complete - returned to center");
}
