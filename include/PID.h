#pragma once
#include "cmath"
#include "vex.h"
#include "motors.h"

/*namespace _PID {
    extern double position;
    extern double error;
    extern double i; // integral
    extern double d;
    extern int target;
    extern double kp;
    extern double ki;
    extern double kd;
    extern double drive;
    extern void runPID();
    extern double prev;
    extern int _time;
    extern bool errorChanging;
}*/

//using namespace _PID;
class PID
{

    double position;
    double error;
    double i; // integral
    double d;
    int target;
    double kp = 2;
    double ki = 0;
    double kd = 1;
    double drive;
    void runPID();
    double prev;
    int _time = 1;
    bool errorChanging = true;
    bool stop = false;
    
public:
    PID();

    void reset();

    void update();

    bool isStopped();

    void stopPID();

    double kpUpdate(double addAmount);

    double kdUpdate(double addAmount);

    void runPID(double targetVal, double timeLimit);
};
