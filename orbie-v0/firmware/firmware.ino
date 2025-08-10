/*
 * Q-Learning with BNO055 IMU State and Human Feedback
 * 
 * This implements Q-learning where:
 * - States are determined by BNO055 IMU heading readings
 * - Actions are: left, right, forward (servo movements)
 * - Rewards come from human feedback via button: -1, 0, or +1
 * - Auto-assigns neutral reward (0) after 5 minutes of no feedback
 * 
 * Memory usage: ~300 bytes
 * 
 * Hardware: BNO055 IMU, 2 servos, button, LED
 * Actions: 0=forward, 1=left, 2=right
 */

#include "controller.h"
#include "config.h"
#include "learning.h"

// Global controller instance (declared before the class that uses it)
Controller controller;

// Global Q-learning instance
QLearningOrbie q_agent;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  // Initialize hardware
  controller.beginHardware();
  controller.setLed(true);
  controller.setNeutralPosition();  
}

void loop() {
  // Update controller (reads IMU, button, etc.)
  controller.update();
  
  // Check for human feedback
  q_agent.checkHumanFeedback();
  q_agent.checkButtonPress();
  
  // Check for feedback timeout
  q_agent.checkFeedbackTimeout();
  
  // Process learning step if reward is available
  if (q_agent.hasQueryRequest()) {
    // Only run learning step if (a) the time out has been reached, 
    // or (b) the user has double clicked the button to query for a new action
    q_agent.runLearningStep();
  } else {
    // TODO: If there is only 30 seconds left, let's trigger an action animation agitation!
    // Just wait and show countdown
    static unsigned long last_countdown = 0;
    if (millis() - last_countdown >= 30000) {
      q_agent.printCountdown();
      last_countdown = millis();
    }
  }
  
  delay(500);
  // set arms to neutral position
  controller.setNeutralPosition();

} 
