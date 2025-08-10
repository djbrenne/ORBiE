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

// Global controller instance (declared before the class that uses it)
Controller controller;

class QLearningOrbie {
private:
  // Q-learning parameters
  static const int NUM_ACTIONS = 3; // go straight forward, turn left, turn right
  // 64 states with s_t = [s_t, s_t-1, s_t-2]
  static const int NUM_STATES = 64; 
  const float LEARNING_RATE = 0.2;
  const float DISCOUNT_FACTOR = 0.8;
  const float EPSILON = 0.3;        // Exploration rate
  
  // Q-table (64 states x 3 actions) - packed into 4-bit values
  uint8_t q_table[NUM_STATES][2]; // 2 bytes per state (3 actions × 4 bits + 4 bits padding)
  
  // Current state and action
  int current_state;
  int current_action;
  
  // Training parameters
  int episode_count;
  
  // IMU variables
  float current_heading;

  // History variables
  int heading_history[3]; // [s_t, s_t-1, s_t-2]
  int last_state;
  
  // Human feedback variables
  int human_reward;
  float human_reward_sum; // TODO: make this work for holding button down for a while
  bool query_requested;
  unsigned long feedback_wait_start;
  const unsigned long ACTION_DELAY = 2000;        // 2 seconds between actions
  const unsigned long FEEDBACK_TIMEOUT = 300000;  // 5 minutes (300,000 ms) for feedback
  
  // Button state tracking TODO: Double check this works
  bool last_button_state;
  unsigned long last_button_press_time;
  const unsigned long DOUBLE_CLICK_TIME = 500;    // 500ms for double click detection
  
public:
  // Constructor
  QLearningOrbie() {
    current_state = 0;
    current_action = 0;
    episode_count = 0;
    human_reward = 0;
    human_reward_sum = 0.0;
    query_requested = false;
    feedback_wait_start = 0;
    last_button_state = false;
    last_button_press_time = 0;
    
    // Initialize Q-table with small random values
    randomSeed(analogRead(0));
    initializeQTable();
  }
  
  // Read IMU heading using Controller class
  float readHeading() {
    // This will be called from the main loop where controller.update() is called
    return controller.getHeading();
  }
  
  // Convert compass heading to discrete N, E, S, W state 
  int headingToState(float heading) {
    // 4 states (90° intervals)
    // 0 = North (0° to 89.9°)
    // 1 = East (90° to 179.9°)
    // 2 = South (180° to 269.9°)
    // 3 = West (270° to 359.9°)
    int current_heading = (int)(heading / 90.0) % 4;  // N, E, S, W

    // Update heading history
    updateHeadingHistory(current_heading);

    // Calculate state
    int state = heading_history[0] * 16 + heading_history[1] * 4 + heading_history[2];

    return state;
  }

  void updateHeadingHistory(int new_heading) {
    new_heading = (int)(new_heading / 90.0) % 4;
    // Shift history: [2] <- [1], [1] <- [0], [0] <- new
    heading_history[2] = heading_history[1];
    heading_history[1] = heading_history[0];
    heading_history[0] = new_heading;
    
    // Handle first 3 episodes where history isn't complete
    if (episode_count < 3) {
      // Fill missing history slots with current heading
      for (int i = episode_count; i < 3; i++) {
        heading_history[i] = new_heading;
      }
    }
  }
  
  // Initialize Q-table with small random values
  void initializeQTable() {
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
  int chooseAction(int state) {
    if (random(100) < (EPSILON * 100)) {
      // Random action (exploration)
      return random(NUM_ACTIONS);
    } else {
      // Best action (exploitation)
      return getBestAction(state);
    }
  }
  
  // Helper functions for packed Q-table access
  int8_t getQValue(int state, int action) {
    if (action == 0) {
      return (q_table[state][0] >> 4) & 0x0F; // First 4 bits
    } else if (action == 1) {
      return q_table[state][0] & 0x0F; // Last 4 bits of first byte
    } else {
      return (q_table[state][1] >> 4) & 0x0F; // First 4 bits of second byte
    }
  }
  
  void setQValue(int state, int action, int8_t value) {
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
  int getBestAction(int state) {
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
  int executeAction(int action) {
    // TODO: add a function to execute the action; raising arms of the robot
    if (action == 0) {
      // Go forward
      Serial.println("F");
      controller.setRightServo(R_SERVO_MAX_ANGLE);
      controller.setLeftServo(L_SERVO_MAX_ANGLE);
    } else if (action == 1) {
      // Turn left
      Serial.println("L");
      controller.setRightServo(R_SERVO_MAX_ANGLE);
      controller.setLeftServo(L_SERVO_MIN_ANGLE);
    } else {
      // Turn right
      Serial.println("R");
      controller.setRightServo(R_SERVO_MIN_ANGLE);
      controller.setLeftServo(L_SERVO_MAX_ANGLE);
    }
    
    // Wait for action to complete
    delay(1000);
    
    // Read new IMU state
    float new_heading = readHeading();
    int new_state = headingToState(new_heading);
    
    Serial.print("H");
    Serial.print(new_heading, 0);
    Serial.print("S");
    Serial.println(new_state);
    
    return new_state;
  }
  
  // Update Q-value using Q-learning formula
  void updateQValue(int state, int action, float reward, int next_state) {
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
  
  // Check for human feedback via button
  void checkHumanFeedback() {
    if (Serial.available()) {
      char input = Serial.read();
      switch (input) {
        case '1':
          human_reward_sum += 1;
          Serial.println("+1");
          break;
        case '2':
          query_requested = true;
          Serial.println("Q");
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
  
  // Check if feedback timeout has occurred
  bool checkFeedbackTimeout() {
    // TODO: change so learning only triggers if the user has double clicked the button to query for a new action
    if (!query_requested && (millis() - feedback_wait_start) >= FEEDBACK_TIMEOUT) {
      // Auto-assign neutral reward after 5 minutes
      human_reward = 0;
      query_requested = true;
      Serial.println("Feedback timeout! Auto-assigning neutral reward (0)");
      return true;
    }
    return false;
  }
  
  // Reset training
  void resetTraining() {
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
  void runLearningStep() {
    unsigned long current_time = millis();
    
    // Read current IMU state
    float current_heading_degrees = readHeading();
    current_state = headingToState(current_heading_degrees);
    
    // Update Q-value with previous state-action-reward
    if (human_reward_sum > 0) {
      Serial.print("Q");
      Serial.println(human_reward_sum);
      // Scale the reward sum between 0-1
      human_reward = scaleHumanRewardSum(human_reward_sum);
      Serial.print("S");
      Serial.println(human_reward, 2);
      human_reward_sum = 0;
    } else {
      human_reward = 0;
    }

    updateQValue(last_state, current_action, human_reward, current_state);
    
    // Reset reward flag and turn off LED
    query_requested = false;
    controller.setLed(false);

    // Update episode statistics
    human_reward = 0;
    episode_count++;
    
    Serial.print("E");
    Serial.print(episode_count);
    Serial.print("S");
    Serial.print(last_state);
    Serial.print("A");
    Serial.print(current_action);
    Serial.print("S");
    Serial.print(current_state);
    Serial.print("R");
    Serial.println(human_reward);
    
    Serial.println("W");

    // Choose and execute new action
    current_action = chooseAction(current_state);
    int next_state = executeAction(current_action);
    
    // Store current state and action for next update
    last_state = current_state;
    current_action = current_action;
    
    // Start feedback timer
    feedback_wait_start = current_time;
    
    Serial.println("F");
  }
  
  // Print Q-table (for debugging)
  void printQTable() {
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
  int getMemoryUsage() {
    return NUM_STATES * NUM_ACTIONS + 100;  // Q-table + other variables
  }
  
  // Get training statistics - ULTRA COMPACT VERSION
  void printStats() {
    Serial.print("E");
    Serial.print(episode_count);
    Serial.print("R");
    Serial.print("H");
    Serial.print(human_reward_sum);
    
    // State and history
    Serial.print("S");
    Serial.print(current_state);
    const char* d = "NESW";
    Serial.print(d[heading_history[0]]);
    Serial.print(d[heading_history[1]]);
    Serial.println(d[heading_history[2]]);
    
    // Action and Q-value
    const char* a = "FLR";
    Serial.print("A");
    Serial.print(a[current_action]);
    Serial.print("Q");
    Serial.println(getQValue(current_state, current_action));
    
    // Q-values array
    Serial.print("Q");
    for (int i = 0; i < NUM_ACTIONS; i++) {
      Serial.print(getQValue(current_state, i));
    }
    Serial.println();
    
    // Best action
    Serial.print("B");
    Serial.println(a[getBestAction(current_state)]);
    
    // Timer
    unsigned long t = getRemainingFeedbackTime();
    if (t > 0) {
      Serial.print("T");
      Serial.print(t / 60000);
      Serial.print(":");
      if ((t % 60000) / 1000 < 10) Serial.print("0");
      Serial.println((t % 60000) / 1000);
    } else {
      Serial.println("TR");
    }
    
    // Heading
    Serial.print("H");
    Serial.println(readHeading(), 0);
  }
  
  // Get remaining time for feedback
  unsigned long getRemainingFeedbackTime() {
    if (query_requested) return 0;
    unsigned long elapsed = millis() - feedback_wait_start;
    if (elapsed >= FEEDBACK_TIMEOUT) return 0;
    return FEEDBACK_TIMEOUT - elapsed;
  }
  
  // Print countdown timer
  void printCountdown() {
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
  float scaleHumanRewardSum(float reward_sum) {
    // Use sigmoid function to scale between 0-1
    // This provides smooth scaling and handles any positive value
    return 1.0 / (1.0 + exp(-reward_sum + 1.0));
  }
  
  // Alternative: Simple linear scaling with a maximum cap
  float scaleHumanRewardSumLinear(float reward_sum, float max_reward = 5.0) {
    // Cap the maximum reward and scale linearly
    float capped_reward = min(reward_sum, max_reward);
    return capped_reward / max_reward;
  }

  // Check if we have a query request to process
  bool hasQueryRequest() {
    return query_requested;
  }
};

// Global Q-learning instance
QLearningOrbie q_agent;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  
  Serial.println("QL");
  Serial.println("A:0=F,1=L,2=R");
  Serial.println("S:4dir+2hist");
  Serial.println("R:0,+1");
  
  // Initialize hardware
  controller.beginHardware();
  
  Serial.print("M");
  Serial.println(q_agent.getMemoryUsage());
  
  Serial.println("C:1=+,2=Q,s=S,p=P,r=R");
  
  Serial.println("S");
}

void loop() {
  // Update controller (reads IMU, button, etc.)
  controller.update();
  
  // Check for human feedback
  q_agent.checkHumanFeedback();
  
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
  
  delay(100);
} 