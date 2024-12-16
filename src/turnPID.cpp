#include "cmath"
#include "vex.h"
#include "motors.h"
#include "turnPID.h"
#include <iostream>


// Try: Increasing kD, make integral = 0 when changes signs and increase end error value

    turnPID::turnPID() {
    }

    /*void reset(vex::motor_group Left, vex::motor_group Right)
    {
        error = 0;
        prev = 0;
        i = 0;
        Left.resetPosition();
        Right.resetPosition();
    }*/

    

    void turnPID::tpUpdate()
    {
        position = InertialSensor.heading();


        error = target - position;

        if (error > 180) {
            //printToConsole(error << " is greater than 180");
            error -= 360;
        } else if (error < -180) {
            //printToConsole(error << " is less than -180");
            error += 360;
        }

        
        //std::cout<<abs(target - position)<<std::endl;
        printToConsole(error);
        // std::cout<<error<<std::endl;

        Brain.Screen.clearLine();
        Brain.Screen.print(error);

        i = i + error + (prev - error) / 2.0;
        d = error - prev;
        prev = error;
        if (error / fabs(error) != prev / fabs(prev))
        {
            i = 0;
        }

        if (fabs(i) >= 100)
        {
            i = (i / fabs(i)) * 100;
        }
        

    }

    void turnPID::runTurnPID(double targetVal/*, vex::motor_group Left, vex::motor_group Right*/)
    {
        int time = 0;
        // std::cout<<position<<std::endl;
        position = 0;
        //InertialSensor.resetHeading();
        target = targetVal;
        // std::cout<<position<<std::endl;
        // wait(1, vex::sec);
        if (target > 180) {
            target -= 360;
        } else if (target < -180) {
            target += 360;
        }
        while (fabs(target - position) > 8) {
            tpUpdate();
            // std::cout<<"h"<<std::endl;
            // std::cout<<position<<std::endl;
            //spinAll(true, (kp * error) + (ki * i) + (kd * d));
            Left.spin(vex::forward, (kp * error) + (ki * i) + (kd * d), vex::pct);
            Right.spin(vex::reverse, (kp * error) + (ki * i)  + (kd * d), vex::pct);
            // tpUpdate();

            if (position > 360) {
                position -= 360;
            } else if (position < -360) {
                position += 360;
            }
            
            time += 20;
            vex::wait(20, vex::msec);
            if (time >= 2000) {
                break;
            }

            // if (fabs(target - position) <= 4) {
            //     error /= 2;
            // }
        } 
        Left.stop(vex::brake);
        Right.stop(vex::brake);
        std::cout<<"done :)"<<std::endl;

        vex::wait(20, vex::msec);
        
    }