/*
    ORBiE v0 Firmware Learning
    Copyright (c) 2025 Laura Petrich

    This file contains the implementation of the learning functionality for the ORBiE v0 firmware.
*/

#include "learning.h"
#include <Arduino.h>

// External controller instance (declared in firmware.ino)
extern Controller controller;

// Constructor
QLearningOrbie::QLearningOrbie() {
    current_state = 0;
    current_action = 0;
    episode_count = 0;
    human_reward = 0;
    human_reward_sum = 0.0;
    query_requested = false;
    feedback_wait_start = 0;
    last_button_state = false;
    last_button_press_time = 0;
    
    // Initialize heading history to prevent random memory values
    heading_history[0] = 0;
    heading_history[1] = 0;
    heading_history[2] = 0;
    
    // Initialize Q-table with small random values
    randomSeed(analogRead(0));
    initializeQTable();
}

// Convert compass heading to discrete N, E, S, W state 
int QLearningOrbie::headingToState() {
    // 4 states (90° intervals)
    // 0 = North (0° to 89.9°)
    // 1 = East (90° to 179.9°)
    // 2 = South (180° to 269.9°)
    // 3 = West (270° to 359.9°)
    
    int state = heading_history[0] * 16 + heading_history[1] * 4 + heading_history[2];
    return state;
}

  void QLearningOrbie::updateHeadingHistory(int new_heading) {
    // For the first 3 episodes, gradually build up the history
    if (episode_count < 3) {
      if (episode_count == 0) {
        // First episode: just set the current heading
        heading_history[0] = new_heading;
        // Keep [1] and [2] as 0 (initialized in constructor)
      } else if (episode_count == 1) {
        // Second episode: shift and add new heading
        heading_history[1] = heading_history[0];  // Move last heading to position 1
        heading_history[0] = new_heading;         // Set new heading at position 0
        // Keep [2] as 0
      } else if (episode_count == 2) {
        // Third episode: shift and add new heading
        heading_history[2] = heading_history[1];  // Move last heading to position 2
        heading_history[1] = heading_history[0];  // Move last_last heading to position 1
        heading_history[0] = new_heading;         // Set new heading at position 0
      }
    } else {
      // Normal operation: shift all history and add new heading
      heading_history[2] = heading_history[1];
      heading_history[1] = heading_history[0];
      heading_history[0] = new_heading;
    }
  }
  
  // Initialize Q-table with small random values
  void QLearningOrbie::initializeQTable() {
    for (int s = 0; s < NUM_STATES; s++) {
      // Pack 3 actions into 2 bytes: [action0:4][action1:4] [action2:4][padding:4]
      uint8_t action0 = (random(11) - 5) & 0x0F; // 4 bits, range -5 to 5
      uint8_t action1 = (random(11) - 5) & 0x0F;
      uint8_t action2 = (random(11) - 5) & 0x0F;
      
      q_table[s][0] = (action0 << 4) | action1;
      q_table[s][1] = (action2 << 4) | 0x00; // padding
    }
  }
  
  // Choose action using epsilon-greedy policy
  int QLearningOrbie::chooseAction(int state) {
    if (random(100) < (EPSILON * 100)) {
      // Random action (exploration)
      // TODO: Maybe show this with leds
      Serial.println("Exploring");
      return random(NUM_ACTIONS);
    } else {
      // Best action (exploitation)
      Serial.println("Exploiting");
      return getBestAction(state);
    }
  }
  
  // Helper functions for packed Q-table access
  int8_t QLearningOrbie::getQValue(int state, int action) {
    if (action == 0) {
      return (q_table[state][0] >> 4) & 0x0F; // First 4 bits
    } else if (action == 1) {
      return q_table[state][0] & 0x0F; // Last 4 bits of first byte
    } else {
      return (q_table[state][1] >> 4) & 0x0F; // First 4 bits of second byte
    }
  }
  
  void QLearningOrbie::setQValue(int state, int action, int8_t value) {
    value = value & 0x0F; // Ensure 4-bit range
    if (action == 0) {
      q_table[state][0] = (q_table[state][0] & 0x0F) | (value << 4);
    } else if (action == 1) {
      q_table[state][0] = (q_table[state][0] & 0xF0) | value;
    } else {
      q_table[state][1] = (q_table[state][1] & 0x0F) | (value << 4);
    }
  }
  
  // Get best action for a state
  int QLearningOrbie::getBestAction(int state) {
    int best_action = 0;
    int8_t best_value = getQValue(state, 0);
    
    for (int a = 1; a < NUM_ACTIONS; a++) {
      int8_t current_value = getQValue(state, a);
      if (current_value > best_value) {
        best_value = current_value;
        best_action = a;
      }
    }
    return best_action;
  }
  
  // Execute action and get new state
  int QLearningOrbie::executeAction(int action) {
    // TODO: add a function to execute the action; raising arms of the robot
    if (action == 0) {
      // Go forward
      Serial.println("Keep going straight");
      controller.setRightServo(90);
      controller.setLeftServo(90);
    } else if (action == 1) {
      // Turn left
      Serial.println("Take the next left turn");
      controller.setRightServo(10);
      controller.setLeftServo(10);
    } else {
      // Turn right
      Serial.println("Take the next right turn");
      controller.setRightServo(170);
      controller.setLeftServo(170);
    }
    
    // Wait for action to complete
    delay(1000);
    
    int new_state = headingToState();
    
    return new_state;
  }
  
  // Update Q-value using Q-learning formula
  void QLearningOrbie::updateQValue(int state, int action, float reward, int next_state) {
    Serial.print("Updating Q-value for state: ");
    Serial.print(state);
    Serial.print(" action: ");
    Serial.print(action);
    Serial.print(" reward: ");
    Serial.print(reward);
    Serial.print(" next_state: ");
    Serial.println(next_state);
    // Get current Q-value
    float current_q = getQValue(state, action) / 10.0;
    
    // Get max Q-value for next state
    float max_next_q = getQValue(next_state, 0) / 10.0;
    for (int a = 1; a < NUM_ACTIONS; a++) {
      float next_q = getQValue(next_state, a) / 10.0;
      if (next_q > max_next_q) {
        max_next_q = next_q;
      }
    }
    
    // Q-learning update formula
    float new_q = current_q + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max_next_q - current_q);
    
    // Store back in Q-table (scaled by 10, constrained to 4-bit range)
    int8_t scaled_value = constrain(new_q * 10, -7, 7); // 4-bit signed range: -7 to 7
    setQValue(state, action, scaled_value);
  }
// Reset training
void QLearningOrbie::resetTraining() {
    Serial.println("Resetting training...");
    initializeQTable();
    episode_count = 0;
    
    // Reset servos to down
    controller.setRightServo(R_SERVO_MIN_ANGLE);
    controller.setLeftServo(L_SERVO_MIN_ANGLE);
    
    // Turn off LED
    controller.setLed(false);
  }
  
  // Run one learning step
  void QLearningOrbie::runLearningStep() {
    unsigned long current_time = millis();
    
    // Read current IMU state
    float current_heading_degrees = controller.getHeading();
    int current_heading = (int)(current_heading_degrees / 90.0) % 4;  // N, E, S, W
    updateHeadingHistory(current_heading);
    current_state = headingToState();

    // return early if we are in the first 3 episodes as our state will not be valid
    if (episode_count < 3) {
      Serial.println("Skipping learning step as we are in the first 3 episodes");
      Serial.print("Heading History: ");
      const char* directions = "NESW";
      Serial.print(directions[heading_history[0]]);
      Serial.print(directions[heading_history[1]]);
      Serial.println(directions[heading_history[2]]);
    } else {
        // Print current status
        Serial.print("Heading: ");
        Serial.print(current_heading_degrees, 0);
        Serial.print("° (");
        const char* directions = "NESW";
        Serial.print(directions[current_heading]);
        Serial.print(") | History: ");
        Serial.print(directions[heading_history[0]]);
        Serial.print(directions[heading_history[1]]);
        Serial.print(directions[heading_history[2]]);
        Serial.print(" | Reward: ");
        Serial.print(human_reward_sum, 2);

        // Update Q-value with previous state-action-reward
        if (human_reward_sum > 0) {
        human_reward = scaleHumanRewardSum(human_reward_sum);
        } else {
        human_reward = 0;
        }
        Serial.print(" | Scaled Reward: ");
        Serial.print(human_reward, 2);
        Serial.println();   

        updateQValue(last_state, current_action, human_reward, current_state); 
        
        // Choose and execute new action
        current_action = chooseAction(current_state);
        int next_state = executeAction(current_action);
    }
    // Reset reward flag and turn off LED
    query_requested = false;
    controller.setLed(false);

    // Update episode statistics
    human_reward = 0;
    human_reward_sum = 0;
    episode_count++;

    // Store current state and action for next update
    last_state = current_state;
    current_action = current_action;
    
    // Start feedback timer
    feedback_wait_start = current_time;    
  }
  
  // Print Q-table (for debugging)
  void QLearningOrbie::printQTable() {
    Serial.println("\nQ-Table (scaled by 10):");
    Serial.println("State\\Action | Left | Right|Forward");
    Serial.println("-------------|------|------|-------");
    
    for (int s = 0; s < NUM_STATES; s++) {
      Serial.print("State ");
      Serial.print(s);
      Serial.print(" (");
      Serial.print(s * 45);
      Serial.print("°) | ");
      
      for (int a = 0; a < NUM_ACTIONS; a++) {
        Serial.print(getQValue(s, a));
        if (a < NUM_ACTIONS - 1) Serial.print(" | ");
      }
      Serial.println();
    }
  }
  
  // Get memory usage
  int QLearningOrbie::getMemoryUsage() {
    return NUM_STATES * NUM_ACTIONS + 100;  // Q-table + other variables
  }
  
  // Get training statistics - ULTRA COMPACT VERSION
  void QLearningOrbie::printStats() {
    Serial.print("Current human reward: ");
    Serial.println(human_reward_sum, 2);
    Serial.print("Human reward scaled: ");
    Serial.println(scaleHumanRewardSum(human_reward_sum), 2);
    
    // State and history
    Serial.print("Current heading: ");
    Serial.println(controller.getHeading(), 0);
    Serial.print("Heading to state: ");
    Serial.println(headingToState());
    Serial.print("Current state: ");
    Serial.println(current_state);
    Serial.print("Current heading history :");
    const char* d = "NESW";
    Serial.print(d[heading_history[0]]);
    Serial.print(d[heading_history[1]]);
    Serial.println(d[heading_history[2]]);
    
    // Action and Q-value
    const char* a = "FLR";
    Serial.print("Current action: ");
    Serial.println(a[current_action]);
    // Q-values array
    Serial.print("Q-values: ");
    for (int i = 0; i < NUM_ACTIONS; i++) {
      Serial.print(getQValue(current_state, i));
      Serial.print(" ");
    }
    Serial.println();
    
  }
  
  // Get remaining time for feedback
  unsigned long QLearningOrbie::getRemainingFeedbackTime() {
    if (query_requested) return 0;
    unsigned long elapsed = millis() - feedback_wait_start;
    if (elapsed >= FEEDBACK_TIMEOUT) return 0;
    return FEEDBACK_TIMEOUT - elapsed;
  }
  
  // Print countdown timer
  void QLearningOrbie::printCountdown() {
    unsigned long remaining = getRemainingFeedbackTime();
    if (remaining > 0) {
      unsigned long minutes = remaining / 60000;
      unsigned long seconds = (remaining % 60000) / 1000;
      Serial.print("Time remaining: ");
      Serial.print(minutes);
      Serial.print(":");
      if (seconds < 10) Serial.print("0");
      Serial.println(seconds);
    }
  }

  // Scale human reward sum between 0-1 using sigmoid function
  float QLearningOrbie::scaleHumanRewardSum(float reward_sum) {
    // Use sigmoid function to scale between 0-1
    // This provides smooth scaling and handles any positive value
    return 1.0 / (1.0 + exp(-reward_sum + 1.0));
  }
  
  // Alternative: Simple linear scaling with a maximum cap
  float QLearningOrbie::scaleHumanRewardSumLinear(float reward_sum, float max_reward) {
    // Cap the maximum reward and scale linearly
    float capped_reward = min(reward_sum, max_reward);
    return capped_reward / max_reward;
  }

  // Check if we have a query request to process
  bool QLearningOrbie::hasQueryRequest() {
    return query_requested;
  }

// Check for human feedback via button
void QLearningOrbie::checkHumanFeedback() {
    if (Serial.available()) {
        char input = Serial.read();
        
        switch (input) {
            case '1':
                human_reward_sum += 1;
                Serial.println("+1 reward added");
                break;
            case 'q':
            case 'Q':
                query_requested = true;
                Serial.println("Query requested");
                break;
            case 's':
            case 'S':
                printQTable();
                break;
            case 'p':
            case 'P':
                printStats();
                break;
            case 'r':
            case 'R':
                resetTraining();
                break;
        }
    }
}

// Check for button press
void QLearningOrbie::checkButtonPress() {
    while (digitalRead(BUTTON_PIN) == LOW) {
        // while button is HIGH, increment the reward sum continuously
        human_reward_sum += 0.01;
        delay(10);
    }
}

// Check if feedback timeout has occurred
bool QLearningOrbie::checkFeedbackTimeout() {
    // TODO: change so learning only triggers if the user has double clicked the button to query for a new action
    if (!query_requested && (millis() - feedback_wait_start) >= FEEDBACK_TIMEOUT) {
        // Auto-assign neutral reward after 5 minutes
        human_reward = 0;
        query_requested = true;
        Serial.println("Feedback timeout!");
        return true;
    }
    return false;
}
