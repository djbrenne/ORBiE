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
  
  // Check for debug requests
  q_agent.checkDebugRequest();

  // Check for rewards and action requests
  q_agent.checkButtonPress();
  q_agent.checkUnpromptedActionTimeout();
  
  // If it's action time, run learning step
  if (q_agent.shouldAct()) {
    // Collect all outstanding rewards
    q_agent.collectReward();
    
    // Run learning step
    q_agent.runLearningStep();
    
    // Let chosen action animate for a second. TODO: proper animations
    delay(1000);
  }

  // set arms to neutral position
  controller.setNeutralPosition();

  // short delay to not overwhelm the controller
  delay(100);

} 
