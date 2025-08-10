# ORBiE v0 Firmware

Firmware for ORBiE v0. This firmware provides methods for sensor and actuator management, learning, and control.

## Overview

ORBiE v0 firmware is designed to run on Arduino-compatible hardware and provides:
- **Learning functionality** - Adaptive behavior and memory persistence
- **Action animations** - Expressive movements and interactions
- **Modular architecture** - Clean separation of concerns

## File Structure

```
firmware/
├── main.ino              # Main entry point and Arduino setup
├── config.h              # Configuration constants and settings
├── learning.h            # Learning system header
├── learning.cpp          # Learning system implementation
├── action-animations.h   # Animation system header
└── action-animations.cpp # Animation system implementation
```

## Main Loop Architecture

The main loop follows a **state machine pattern** with **non-blocking timing** to ensure responsive interactions while maintaining expressive animations. The device is usually idle, but occasionally enters a learning state where it updates its learning parameters based on reward collected while idle, and chooses a new action. The interval between each learning step may take an arbitrary amount of time up to `MAX_ACTION_INTERVAL`, and is initiated either by a human query or an internal timer.

### Core Components

```cpp
// Setup structure
void setup() {
    // Initialize hardware
    beginHardware();
    // Initialize learning parameters
    s_t, w = beginLearning();
    reward = 0;
    // Choose initial action
    action = chooseAction(s_t, w);
    // Take initial action
    takeAction(action);
}

// Main loop structure
void loop() {
    unsigned long currentTime = millis();
    
    // 1. Sensor Reading (every cycle)
    int direction = readSensors();
    
    // 2. Button Handling (every cycle) - blocks until button release
    struct ButtonResult result = checkButton();
    bool is_query = result.is_query;
    reward += result.press_duration;
    
    // 3. Animation System (non-blocking)
    updateAnimations(currentTime);

    // 4. State Machine Update (every cycle) - handles timing checks for unprompted actions
    int machine_state = updateStateMachine(currentTime, next_action_time);
    
    // 5. Learning and Acting System (for both queries and unprompted actions)
    bool should_act = (is_query || (machine_state == LEARNING));
    
    if (should_act) {
        animate(THINKING); // Must be non-blocking
        w = updateLearning(reward); // Assign all reward since the last action to the last action
        reward = 0; // Re-initialize reward counter
        s_t = representState(is_query); // What direction are you facing? Is this a query answer or unprompted action?
        action = chooseAction(s_t, w); // Choose epsilon-greedy (or other type) of action
        takeAction(action); 
        last_action_time = currentTime;
        // Whether unprompted or queried action, reset the next unprompted action to some random future time
        next_action_time = currentTime + random(MIN_ACTION_INTERVAL, MAX_ACTION_INTERVAL);
    }
    
    // 6. Safety & System Health (periodic)
    if (currentTime - lastHealthCheck >= HEALTH_CHECK_INTERVAL) {
        checkSystemHealth();
        lastHealthCheck = currentTime;
    }
}
```

### State Machine States

1. **IDLE** - Default expressive state, responsive to queries
2. **LEARNING** - Taking an unprompted action and observing the result
3. **ERROR** - Safe state when issues detected

### Sensor Integration

- **Head Button** - Debounced input for reward signals and queries
- **Magnetometer** - Orientation sensing for expressive behaviors
- **Battery Monitor** - Power management and status indication

### Animation System

- **LED Eyes** - Blinking, color modulation, brightness control
- **Flipper Servos** - Expressive gestures, yes/no/dunno responses, idle motion

### Learning Integration

- **State Representation** - Cardinal direction (int, 0-3), Query Available (bool)
- **Reward Processing** - Head button duration → reward signal (int)
- **Behavior Adaptation** - Adjusting responses based on learned preferences
- **Memory Persistence** - SD card logging for cross-power-cycle learning

### Timing Considerations

- **Main Loop**: ~100Hz (10ms cycle time)
- **Animation Updates**: 30-60Hz for smooth motion
- **Learning Updates**: 1-5Hz for background processing
- **Health Checks**: 1Hz for system monitoring

### Safety Features

- **Servo Limits** - Prevent over-rotation and damage
- **Battery Protection** - Low-power modes and warnings
- **Error Recovery** - Graceful degradation on sensor failures
- **Non-blocking Design** - Responsive even during long operations

## Development Status

⚠️ **Early Development** - This firmware is currently in early development stages. Most files contain only basic structure and headers.

## Building and Flashing

### Prerequisites
- Arduino IDE or compatible development environment
- Arduino Leonardo

### Build Steps
1. Open `main.ino` in Arduino IDE
2. Select appropriate board and port
3. Verify and upload

### Configuration
Edit `config.h` to customize:
- Pin assignments
- Animation timing
- Learning parameters
- System behavior

## Contributing

This firmware is part of the RoadBot art project. Contributions should align with the project's artistic and technical goals.

### Code Style
- Follow Arduino C++ conventions
- Use descriptive variable and function names
- Include appropriate comments for complex logic
- Maintain modular structure

## License

This firmware is part of the RoadBot project and is licensed under the Artistic Use License. See the main project LICENSE.md for details.

## Authors

- **Dylan Brenneis** - Main firmware architecture, action animations
- **Laura Petrich** - Learning system implementation

## Future Development

Planned features for upcoming versions:
- [ ] Complete learning algorithm implementation
- [ ] Full animation library
- [ ] Hardware sensor integration
- [ ] Wireless communication
- [ ] Safety systems
- [ ] Persistent memory implementation
