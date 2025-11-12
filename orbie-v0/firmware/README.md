# ORBiE v0 Firmware

Firmware for ORBiE v0. This firmware implements a Q-learning agent that learns navigation behaviors through human feedback.

## Overview

ORBiE v0 firmware implements a Q-learning reinforcement learning system where:
- **States** are determined by BNO055 IMU heading readings (cardinal directions: N, E, S, W)
- **Actions** are: forward, turn left, turn right (servo movements)
- **Rewards** come from human feedback via button press duration
- **Learning steps** are triggered by human query (double-click) or an internal timer with potentiometer-controlled timeout

## File Structure

```
firmware/
├── firmware.ino        # Main entry point and Arduino setup/loop
├── config.h            # Configuration constants and pin definitions
├── controller.h         # Controller class header (hardware abstraction)
├── controller.cpp       # Controller class implementation
├── learning.h           # Q-learning system header
└── learning.cpp         # Q-learning system implementation
```

## Main Loop Architecture

The main loop follows a simple event-driven pattern:

```cpp
void loop() {
  // Update controller (reads IMU, button, etc.)
  controller.update();
  
  // Check for debug requests via serial
  q_agent.checkDebugRequest();

  // Check for rewards and action requests
  q_agent.checkButtonPress();
  q_agent.checkUnpromptedActionTimeout();
  
  // If it's action time, run learning step
  if (q_agent.shouldAct()) {
    q_agent.collectReward();
    q_agent.runLearningStep();
    delay(1000);  // Let action animate
  }

  // Return to neutral position
  controller.setNeutralPosition();
  delay(100);
}
```

### Learning System

The Q-learning agent uses:
- **64 states**: Based on heading history (current + 2 previous headings, each with 4 cardinal directions)
- **3 actions**: Forward, turn left, turn right
- **Epsilon-greedy exploration**: 70% exploration rate
- **Q-table**: Packed into 4-bit values for memory efficiency
- **Reward scaling**: Sigmoid function to scale button press duration to 0-1 range

### Action Timing

Actions are triggered by:
1. **Human query**: Double-clicking the button immediately triggers a learning step
2. **Unprompted timeout**: A timer that expires after a random interval sampled from a log-uniform distribution

The timeout interval is controlled by a **potentiometer** on pin A0:
- **Low pot (0.0)**: Timeout range 1-5 seconds
- **High pot (1.0)**: Timeout range 15-60 minutes
- **Formula**: Uses exponential interpolation with log-uniform sampling between bounds

### Button Interaction

- **Single click**: Accumulates reward based on press duration (1ms = 0.001 reward)
- **Double click**: Triggers immediate action/learning step (query mode)
- **Debouncing**: 20ms debounce delay
- **Double-click window**: 300ms

## Hardware Components

### Implemented Hardware

- **BNO055 IMU** (I2C): Orientation sensing for heading-based states
- **Two Servos** (PWM): Left and right servos for movement actions
- **Button** (Digital, interrupt-enabled): Reward input and query trigger
- **WS2812B LED** (PWM): Status indication (color indicates exploration vs exploitation)
- **Potentiometer** (Analog, A0): Controls unprompted action timeout range

### Pin Assignments

Defined in `config.h`:
- Button: Pin 2 (with interrupt)
- LED: Pin 9
- Right Servo: Pin 5
- Left Servo: Pin 6
- Potentiometer: Pin A0
- BNO055: I2C (address 0x55)

## Controller Class

The `Controller` class provides hardware abstraction:

### Button Functions
- `checkButtonEvents()`: Returns reward and query status
- `isButtonPressed()`: Debounced button state
- `wasButtonPressed()`: Edge detection

### Servo Functions
- `setRightServo(angle)`, `setLeftServo(angle)`: Set servo positions
- `goForward()`, `turnLeft()`, `turnRight()`: Predefined action movements
- `setNeutralPosition()`: Return to neutral pose

### IMU Functions
- `updateImuData()`: Read latest sensor data
- `getHeading()`, `getPitch()`, `getRoll()`: Orientation angles
- `getAngularVelocityX/Y/Z()`: Angular velocity
- `getLinearAccelX/Y/Z()`: Linear acceleration

### LED Functions
- `setLed(state)`: Turn LED on/off
- `setLedColor(r, g, b)`: Set RGB color
- Colors indicate learning mode: Green = exploration, Blue = exploitation

### Potentiometer Functions
- `getPotentiometerValue()`: Returns normalized value (0.0-1.0)
- `calculateUnpromptedActionTimeout()`: Calculates timeout using log-uniform sampling
- `getTimeoutBounds(L, U)`: Returns calculated bounds for testing

## Q-Learning Agent

The `QLearningOrbie` class implements the learning system:

### Key Parameters
- Learning rate: 0.2
- Discount factor: 0.8
- Exploration rate (epsilon): 0.7
- Q-table: 64 states × 3 actions (packed into 4-bit values)

### State Representation
States are encoded from heading history:
- Each heading is discretized to 4 cardinal directions (N=0, E=1, S=2, W=3)
- State = `heading[0] * 16 + heading[1] * 4 + heading[2]`
- First 3 episodes build up history gradually

### Reward Processing
- Button press duration → reward sum
- Reward sum → scaled reward (0-1) via sigmoid function
- Scaled reward used in Q-learning update

### Debug Features
Serial commands:
- `1`: Add +1 reward
- `q`/`Q`: Trigger action immediately
- `s`/`S`: Print Q-table
- `p`/`P`: Print statistics
- `r`/`R`: Reset training

## Configuration

Edit `config.h` to customize:

### Pin Definitions
- Button, LED, servo pins
- Potentiometer pin
- IMU I2C address

### Timing Parameters
- Serial baud rate
- Debounce delay
- Double-click detection window

### Learning Parameters
- Unprompted action timeout bounds (in seconds):
  - `UNPROMPTED_ACTION_TIMEOUT_MIN`: Lower bound minimum (at pot=0)
  - `UNPROMPTED_ACTION_TIMEOUT_MAX`: Upper bound maximum (at pot=1)
  - `UNPROMPTED_ACTION_TIMEOUT_L_MAX`: Lower bound maximum (at pot=1)
  - `UNPROMPTED_ACTION_TIMEOUT_U_MIN`: Upper bound minimum (at pot=0)

### Servo Limits
- Min/max angles for each servo to prevent over-rotation

## Building and Flashing

### Prerequisites
- Arduino IDE or compatible development environment
- Arduino Nano or compatible board
- Required libraries:
  - `Servo` (built-in)
  - `Wire` (built-in)
  - `Adafruit_Sensor`
  - `Adafruit_BNO055`
  - `FastLED`

### Build Steps
1. Install required libraries via Arduino Library Manager
2. Open `firmware.ino` in Arduino IDE
3. Select appropriate board (Arduino Nano) and port
4. Verify and upload

### Serial Monitor
Open serial monitor at 115200 baud to see:
- Hardware initialization status
- Learning step information
- Q-table and statistics (via debug commands)
- Heading and reward information

## Development Status

✅ **Functional** - Core learning system is implemented and operational.

### Implemented Features
- ✅ Q-learning algorithm with 64-state, 3-action system
- ✅ Hardware abstraction layer (Controller class)
- ✅ Button-based reward system with double-click queries
- ✅ IMU-based state representation
- ✅ Potentiometer-controlled timeout system
- ✅ Serial debug interface
- ✅ Memory-efficient Q-table packing

### Future Enhancements
- [ ] Persistent memory (SD card logging)
- [ ] More sophisticated animation system
- [ ] Additional sensor integration
- [ ] Wireless communication

## Contributing

This firmware is part of the ORBiE art project. Contributions should align with the project's artistic and technical goals.

### Code Style
- Follow Arduino C++ conventions
- Use descriptive variable and function names
- Include appropriate comments for complex logic
- Maintain modular structure

## License

This firmware is part of the RoadBot project and is licensed under the Artistic Use License. See the main project LICENSE.md for details.

## Authors

- **Dylan Brenneis** - Main firmware architecture, controller implementation
- **Laura Petrich** - Learning system implementation
