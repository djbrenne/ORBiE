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

## Main Loop

TBD

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
