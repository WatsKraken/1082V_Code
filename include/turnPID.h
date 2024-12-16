#pragma once
#ifndef turnPID_H
#define turnPID_H

#include "cmath"
#include "vex.h"
#include "motors.h"
#include <iostream>

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
    double kd = 0.3;
    double drive;
    void runTurnPID();
    double prev;

public:
    turnPID();
    void reset();
    void tpUpdate();
    void runTurnPID(double targetVal/*, vex::motor_group Left, vex::motor_group Right*/);
};

#endif