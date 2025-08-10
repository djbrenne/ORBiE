/*
    ORBiE v0 Firmware Learning
    Copyright (c) 2025 Laura Petrich

    This file contains the learning functionality for the ORBiE v0 firmware.
*/

#ifndef LEARNING_H
#define LEARNING_H

#include "controller.h"
#include "config.h"

// Forward declaration
class Controller;

class QLearningOrbie {
private:
    // Q-learning parameters
    static const int NUM_ACTIONS = 3; // go straight forward, turn left, turn right
    static const int NUM_STATES = 64; // 64 states with s_t = [s_t, s_t-1, s_t-2]
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
    const unsigned long FEEDBACK_TIMEOUT = 60000;  // 5 minutes (300,000 ms) for feedback
    
    // Button state tracking TODO: Double check this works
    bool last_button_state;
    unsigned long last_button_press_time;
    const unsigned long DOUBLE_CLICK_TIME = 500;    // 500ms for double click detection

public:
    // Constructor
    QLearningOrbie();
    
    // Core learning functions
    void runLearningStep();
    void resetTraining();
    bool hasQueryRequest();
    
    // Human feedback functions
    void checkHumanFeedback();
    bool checkFeedbackTimeout();
    
    // Utility functions
    void printQTable();
    void printStats();
    void printCountdown();
    unsigned long getRemainingFeedbackTime();
    int getMemoryUsage();
    
    // State and action functions
    int headingToState();
    void updateHeadingHistory(int new_heading);
    int chooseAction(int state);
    int executeAction(int action);
    int getBestAction(int state);
    
    // Q-table access functions
    int8_t getQValue(int state, int action);
    void setQValue(int state, int action, int8_t value);
    void updateQValue(int state, int action, float reward, int next_state);
    void initializeQTable();
    
    // Reward scaling functions
    float scaleHumanRewardSum(float reward_sum);
    float scaleHumanRewardSumLinear(float reward_sum, float max_reward = 5.0);
};

#endif // LEARNING_H