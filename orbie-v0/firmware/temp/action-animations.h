/*
    ORBiE v0 Firmware Action Animations
    Copyright (c) 2025 Dylan Brenneis

    This file contains the action animations for the ORBiE v0 firmware.
*/

#ifndef ACTION_ANIMATIONS_H
#define ACTION_ANIMATIONS_H

#include "controller.h"

enum RobotAction {
    ACTION_FORWARD     = 0,
    ACTION_LEFT        = 1,
    ACTION_RIGHT       = 2,
    ACTION_AGITATED    = 3,
    ACTION_HAPPY_START = 4,
    ACTION_HAPPY_STOP  = 5,
    ACTION_ARMS_DOWN   = 6
};

void executeActionAnimation(Controller& controller, int action);
void updateHappyAnimation(Controller& controller);

#endif // ACTION_ANIMATIONS_H