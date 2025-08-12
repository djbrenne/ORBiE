/*
 * ORBiE v0 Firmware
 * 
 * This implements Q-learning where:
 * - States are determined by BNO055 IMU heading readings
 * - Actions are: left, right, forward (servo movements)
 * - Positive scalar rewards come from human feedback via button
 * - If no button press is detected between actions, the reward is 0
 * - Action/Learning steps are triggered by a human query or an internal timer
 * - Double-clicking the button triggers a yes/no query.
 *  
 * Hardware: BNO055 IMU, 2 servos, button, LED
 * Actions: 0=forward/dunno, 1=left/no, 2=right/yes
 */

#include "controller.h"
#include "config.h"
#include "learning.h"

// Global controller instance
Controller controller;

// Global Q-learning instance
QLearningOrbie q_agent;

void setup() {
  // Initialize hardware
  controller.beginHardware();
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
  if (q_agent.hasQueryRequest() && !controller.isButtonPressed()) {
    // Only run learning step if the time out has been reached, 
    // or the user has double clicked the button to query for a new action,
    // and the button is not currently being held down (get ALL the rewards!)
    q_agent.runLearningStep();
    delay(1000);
  } else {
    // TODO: If there is only 30 seconds left, let's trigger an action animation agitation!
    // Just wait and show countdown
    static unsigned long last_countdown = 0;
    if (millis() - last_countdown >= 30000) {
      q_agent.printCountdown();
      last_countdown = millis();
    }
  }
  
  delay(100);
  // set arms to neutral position
  controller.setNeutralPosition();

} 
