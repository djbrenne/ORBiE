/*
 * Q-Learning with Magnetometer State and Human Feedback
 * 
 * This implements Q-learning where:
 * - States are determined by magnetometer readings (compass direction)
 * - Actions are: left, right, forward
 * - Rewards come from human feedback: -1, 0, or +1
 * - Auto-assigns neutral reward (0) after 5 minutes of no feedback
 * 
 * Memory usage: ~300 bytes
 * 
 * Hardware: HMC5883L or QMC5883L magnetometer
 * Actions: 0=left, 1=right, 2=forward
 */

#include <Wire.h>

class QLearningOrbie {
private:
  // Q-learning parameters
  const int NUM_ACTIONS = 3; // go straight forward, turn left, turn right
  // 64 states with s_t = [s_t, s_t-1, s_t-2]
  const int NUM_STATES = 64; 
  const float LEARNING_RATE = 0.2;
  const float DISCOUNT_FACTOR = 0.8;
  const float EPSILON = 0.3;        // Exploration rate
  
  // Q-table (24 states x 3 actions)
  int8_t q_table[NUM_STATES][NUM_ACTIONS];
  
  // Current state and action
  int current_state;
  int current_action;
  
  // Training parameters
  int episode_count;
  int max_episodes;
  float total_reward;
  
  // Magnetometer variables
  int magnetometer_address;
  float current_heading;

  // History variables
  int heading_history[3]; // [s_t, s_t-1, s_t-2]
  int last_state;
  
  // Human feedback variables
  int human_reward;
  float human_reward_sum; // TODO: make this work for holding button down for a while
  bool reward_received;
  bool query_requested;
  unsigned long last_action_time;
  unsigned long feedback_wait_start;
  const unsigned long ACTION_DELAY = 2000;        // 2 seconds between actions
  const unsigned long FEEDBACK_TIMEOUT = 300000;  // 5 minutes (300,000 ms) for feedback
  
public:
  // Constructor
  QLearningOrbie() {
    current_state = 0;
    current_action = 0;
    episode_count = 0;
    max_episodes = 100;
    total_reward = 0.0;
    human_reward = 0;
    human_reward_sum = 0.0;
    reward_received = false;
    query_requested = false;
    last_action_time = 0;
    feedback_wait_start = 0;
    
    // Initialize magnetometer (try both common addresses)
    magnetometer_address = 0x1E;  // HMC5883L default
    if (!initMagnetometer()) {
      magnetometer_address = 0x0D;  // QMC5883L default
      initMagnetometer();
    }
    
    // Initialize Q-table with small random values
    randomSeed(analogRead(0));
    initializeQTable();
  }
  
  // Initialize magnetometer
  bool initMagnetometer() {
    Wire.begin();
    Wire.beginTransmission(magnetometer_address);
    Wire.write(0x02);  // Mode register
    Wire.write(0x00);  // Continuous measurement mode
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.println("Magnetometer initialized successfully");
      return true;
    } else {
      Serial.println("Failed to initialize magnetometer");
      return false;
    }
  }
  
  // Read magnetometer and convert to compass heading
  float readMagnetometer() {
    Wire.beginTransmission(magnetometer_address);
    Wire.write(0x03);  // Data register
    Wire.endTransmission();
    
    Wire.requestFrom(magnetometer_address, 6);
    if (Wire.available() >= 6) {
      int16_t x = Wire.read() << 8 | Wire.read();
      int16_t z = Wire.read() << 8 | Wire.read();
      int16_t y = Wire.read() << 8 | Wire.read();
      
      // Calculate heading in degrees
      float heading = atan2(y, x) * 180.0 / PI;
      
      // Normalize to 0-360 degrees
      if (heading < 0) heading += 360.0;
      
      return heading;
    }
    return 0.0;
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
    heading_history[0] = heading_history[1];
    heading_history[1] = heading_history[2];
    heading_history[2] = current_heading;

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
      for (int a = 0; a < NUM_ACTIONS; a++) {
        // Random values between -5 and 5 (scaled by 10)
        q_table[s][a] = (random(11) - 5);
      }
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
  
  // Get best action for a state
  int getBestAction(int state) {
    int best_action = 0;
    int8_t best_value = q_table[state][0];
    
    for (int a = 1; a < NUM_ACTIONS; a++) {
      if (q_table[state][a] > best_value) {
        best_value = q_table[state][a];
        best_action = a;
      }
    }
    return best_action;
  }
  
  // Execute action and get new state
  int executeAction(int action) {
    // TODO: add a function to execute the action; raising arms of the robot
    Serial.print("Executing action: ");
    switch (action) {
      case 0:
        Serial.println("FORWARD");
        // Do not take the next turn, keep going straight forward
        break;
      case 1:
        Serial.println("LEFT");
        // Take the next left turn
        break;
      case 2:
        Serial.println("RIGHT");
        // Take the next right turn
        break;
    }
    
    // Read new magnetometer state
    float new_heading = readMagnetometer();
    int new_state = headingToState(new_heading);
    
    // Update heading history with new direction
    updateHeadingHistory(new_heading);
    
    Serial.print("Heading: ");
    Serial.print(new_heading, 1);
    Serial.print("° -> State: ");
    Serial.println(new_state);
    
    return new_state;
  }
  
  // Update Q-value using Q-learning formula
  void updateQValue(int state, int action, float reward, int next_state) {
    // Get current Q-value
    float current_q = q_table[state][action] / 10.0;
    
    // Get max Q-value for next state
    float max_next_q = q_table[next_state][0] / 10.0;
    for (int a = 1; a < NUM_ACTIONS; a++) {
      float next_q = q_table[next_state][a] / 10.0;
      if (next_q > max_next_q) {
        max_next_q = next_q;
      }
    }
    
    // Q-learning update formula
    float new_q = current_q + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max_next_q - current_q);
    
    // Store back in Q-table (scaled by 10)
    // This is for arduino optimization purposes, as the q_table is an int8_t array (storing floats in memory is expensive)
    q_table[state][action] = constrain(new_q * 10, -127, 127);
  }
  
  // Check for human feedback via Serial
  void checkHumanFeedback() {
    if (Serial.available()) {
      char input = Serial.read();
      switch (input) {
        case '0':
          human_reward = 0;
          reward_received = true;
          Serial.println("Reward: 0 (Neutral)");
          break;
        case '1':
          human_reward = 1;
          human_reward_sum += 1;
          reward_received = true;
          Serial.println("Reward: +1 (Good)");
          break;
        case '2':
          // TODO: change this to grab whatever has been sent as the reward up to this point
          human_reward = 0;
          query_requested = true;
          Serial.println("Requesting query: 2");
          break;
        case 's':
        case 'S':
          // Show current Q-table
          printQTable();
          break;
        case 'r':
        case 'R':
          // Reset training
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
    total_reward = 0.0;
  }
  
  // Run one learning step
  void runLearningStep() {
    unsigned long current_time = millis();
    
    // Check if enough time has passed since last action
    if (current_time - last_action_time < ACTION_DELAY) {
      return;
    }
    
    // Read current magnetometer state
    float current_heading_degrees = readMagnetometer();
    current_state = headingToState(current_heading_degrees);
    
    // Update heading history with new reading
    updateHeadingHistory(current_heading_degrees);
    
    // Only proceed if we have a reward from previous action
    if (query_requested) {
      // Update Q-value with previous state-action-reward
      updateQValue(last_state, current_action, human_reward, current_state);
      
      // Reset reward flag
      query_requested = false;
      
      // Update episode statistics
      total_reward += human_reward;
      episode_count++;
      
      Serial.print("Episode ");
      Serial.print(episode_count);
      Serial.print(": State ");
      Serial.print(last_state);
      Serial.print(" -> Action ");
      Serial.print(current_action);
      Serial.print(" -> State ");
      Serial.print(current_state);
      Serial.print(" -> Reward ");
      Serial.println(human_reward);
      
      // Show time remaining for next feedback
      Serial.println("Waiting for human feedback... (5 min timeout)");
    }
    
    // Choose and execute new action
    current_action = chooseAction(current_state);
    int next_state = executeAction(current_action);
    
    // Store current state and action for next update
    last_state = current_state;
    current_action = current_action;
    
    // Update last action time and start feedback timer
    last_action_time = current_time;
    feedback_wait_start = current_time;
    
    // Prompt for human feedback
    Serial.println("Please provide feedback:");
    Serial.println("0: Neutral (0)");
    Serial.println("1: Good (+1)");
    Serial.println("2: Query request for new action");
    Serial.println("s: Show Q-table");
    Serial.println("r: Reset training");
    Serial.println("(Auto-assigns 0 after 5 minutes)");
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
        Serial.print(q_table[s][a]);
        if (a < NUM_ACTIONS - 1) Serial.print(" | ");
      }
      Serial.println();
    }
  }
  
  // Get memory usage
  int getMemoryUsage() {
    return NUM_STATES * NUM_ACTIONS + 100;  // Q-table + other variables
  }
  
  // Get training statistics
  void printStats() {
    Serial.print("Episodes: ");
    Serial.print(episode_count);
    Serial.print(", Total Reward: ");
    Serial.print(total_reward);
    Serial.print(", Avg Reward: ");
    if (episode_count > 0) {
      Serial.println(total_reward / episode_count, 2);
    } else {
      Serial.println("N/A");
    }
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

  // Check if we have a reward to process
  bool hasReward() {
    return reward_received;
  }

  // Check if we have a query request to process
  bool hasQueryRequest() {
    return query_requested;
  }
};

// Global Q-learning instance
QLearningOrbie q_agent;

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("=== Q-Learning with Magnetometer ===");
  Serial.println("Actions: 0=Forward, 1=Left, 2=Right");
  Serial.println("States: 4 compass directions with 2 additional history states");
  Serial.println("Rewards: Human feedback (0, +1)");
  Serial.println("Auto-assigns neutral reward (0) after 5 minutes");
  Serial.print("Memory usage: ");
  Serial.print(q_agent.getMemoryUsage());
  Serial.println(" bytes");
  
  Serial.println("\nCommands:");
  Serial.println("0: Neutral action (0)");
  Serial.println("1: Good action (+1)");
  Serial.println("2: Query request for new action");
  Serial.println("s: Show Q-table");
  Serial.println("r: Reset training");
  
  Serial.println("\nStarting learning...");
}

void loop() {
  // Check for human feedback
  q_agent.checkHumanFeedback();
  
  // Check for feedback timeout
  q_agent.checkFeedbackTimeout();
  
  // Process learning step if reward is available
  if (q_agent.hasReward()) {
    // TODO: only run learning step if (a) the time out has been reached, 
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