#pragma once
#ifndef turnPID_H
#define turnPID_H

#include "cmath"
#include "vex.h"
#include "motors.h"
#include <iostream>

// #define modPos() (position / fabs(position)) * (fabs(position) + pow(fabs(position/10), 1.3) - fabs(position/10)); //Modified position according to a formula

class turnPID
{

// Try: Increasing kD, make integral = 0 when changes signs and increase end error value

    double position;
    double error;
    double i; // integral
    double d;
    int target;
    double kp = 0.5;
    double ki = 0;
    double kd = 0.5;
    double drive;
    double prev;

public:
    turnPID();
    void reset();
    void tpUpdate();
    void stopTurnPID();
    void runTurnPID(double targetVal/*, vex::motor_group Left, vex::motor_group Right*/);
};

#endif