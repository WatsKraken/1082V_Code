/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Joel D.                                                   */
/*    Created:      10/24/2024, 11:18:06 PM                                   */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "motors.h"
#include "PID.h"
#include "turnPID.h"

using namespace vex;
//using namespace _PID;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

PID pid;
turnPID turnPid;
int autonNum;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  RotationSensor.setReversed(true);
  Left.resetPosition();
  Right.resetPosition();
  manlySilverArm.resetPosition();
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
      wait(10, vex::msec);
  }
  InertialSensor.resetHeading();
  
  RotationSensor.resetPosition();

  //Set auton number for each time you upload the program
  autonNum = 10;
}

void spin_for(double time, motor Motor) {
  double timePassed = 0;
  while (timePassed < time) {
    Motor.spin(forward, 80, pct);
    timePassed += time / 1000;
    wait(time, msec);
  }
  Motor.stop(brake);
}

void spin_for_rev(double time, motor Motor, double _pct) {
  double timePassed = 0;
  while (timePassed < time) {
    Motor.spin(reverse, _pct, pct);
    timePassed += time / 1000;
    wait(time, msec);
  }
  Motor.stop(brake);
}

void spinTo(double angle) {
  while (RotationSensor.position(deg) < angle/1.8) {
    manlySilverArm.spin(forward, 10, pct);
  }
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
      wait(10, vex::msec);
  }

  wait(1500, msec);
  
  Brain.Screen.print(autonNum);
  clampPneumatics.set(false);
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  spin_for(0.5, belt);
  if (autonNum == 1) {
    InertialSensor.setHeading(210, degrees);
    // Red left
    pid.runPID(-34.5, 2);
    // turnPid.runTurnPID(150);
    clampPneumatics.set(true);
    belt.spin(fwd, 100, pct);
    pid.runPID(-5,2);
    turnPid.runTurnPID(270/*.690067526*/);
    pid.runPID(24, 2);
    // pid.runPID(2 * sqrt(10), 2);
    Brain.Screen.print("Auton 1 (Red Left) completed.");

  } else if (autonNum == 2) {
    // Red right
    // turnPid.runTurnPID(33.690067526);
    // pid.runPID(43.2/2);
    // belt.spinFor(2, sec)
    InertialSensor.setHeading(150, degrees);
    pid.runPID(-34, 2);
    // turnPid.runTurnPID(150);
    clampPneumatics.set(true);
    belt.spin(fwd, 100, pct);
    pid.runPID(-5,2);
    turnPid.runTurnPID(90/*.690067526*/);
    pid.runPID(24, 2);
    spin_for(2, belt);
    Brain.Screen.print("Auton 2 (Red Right) completed.");
    

  } if (autonNum == 3) {
    // Blue left
    // turnPid.runTurnPID(-33.690067526);
    // pid.runPID(43.2);
    InertialSensor.setHeading(210, degrees);
    pid.runPID(-34, 2);
    // turnPid.runTurnPID(150);
    clampPneumatics.set(true);
    belt.spin(fwd, 100, pct);
    pid.runPID(-5,2);
    turnPid.runTurnPID(270/*.690067526*/);
    pid.runPID(24, 2);
    Brain.Screen.print("Auton 3 (Blue Left) completed.");

  } else if (autonNum == 4) {
    // Blue right
    // turnPid.runTurnPID(33.690067526);
    // pid.runPID(43.2/2);
    // belt.spinFor(2, sec);
    InertialSensor.setHeading(150, degrees);
    pid.runPID(-32, 2);
    // turnPid.runTurnPID(150);
    clampPneumatics.set(true);
    belt.spin(fwd,100,pct);
    pid.runPID(-5,2);
    turnPid.runTurnPID(60/*.690067526*/);
    pid.runPID(24,2);
    pid.runPID(-10,2);
    
    Brain.Screen.print("Auton 4 (Blue Right) completed.");

  } else if (autonNum == 10) {
    //set up backwards
  
    InertialSensor.setHeading(0, degrees);
    clampPneumatics.set(true);
    pid.runPID(-7, 1);
    belt.spin(fwd, 100, pct);
    wait(500, msec);
    belt.stop(hold);
    clampPneumatics.set(false);
    pid.runPID(15,2);
    turnPid.runTurnPID(90);
    pid.runPID(-15,2);
    clampPneumatics.set(true);
    turnPid.runTurnPID(0);
    pid.runPID(11,2);
    belt.spin(fwd, 100, pct);
    turnPid.runTurnPID(-90);
    pid.runPID(9,2);
    pid.runPID(-3,1);
    turnPid.runTurnPID(-90);
    pid.runPID(12,2);
    pid.runPID(8,2);
    turnPid.runTurnPID(-100);
    pid.runPID(24,2);
    belt.stop(hold);
    clampPneumatics.set(false);
    pid.runPID(7, 2);
    turnPid.runTurnPID(80);
    pid.runPID(6,2);
    spin_for(0.5,belt);
    turnPid.runTurnPID(90);
    pid.runPID(64,2);    
    clampPneumatics.set(true);
    belt.spin(fwd, 100, pct);
    turnPid.runTurnPID(180);
    pid.runPID(12,2);
    pid.runPID(9,2);
    pid.runPID(6,2);
    turnPid.runTurnPID(90);
    pid.runPID(16,2);
    belt.stop(hold);
    clampPneumatics.set(false);
    pid.runPID(8,2);

  } else if (autonNum == 5) {
   pid.runPID(24, 2);
    turnPid.runTurnPID(60);
    turnPid.runTurnPID(120);
    turnPid.runTurnPID(150);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                             */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

//in main.cpp
void usercontrol(void) {
  // User control code here, inside the loop
  // simga 🗣️🗣️🔥🔥

  double slow = 1;
  bool pneumaticsBool = false;
  bool pressingBool = false;
  int distance = 0;
  bool mSABool = false;
  bool ready = false;
  int time = 0;
  int time2 = 0;
  bool runTime = false;
  bool runTime2 = false;
  RotationSensor.resetPosition();
  manlySilverArm.resetPosition();

                                                                        /*InertialSensor.calibrate();
                                                                        while (InertialSensor.isCalibrating()) {
                                                                            wait(10, vex::msec);
                                                                        }
                                                                        InertialSensor.resetHeading();

                                                                        wait(1500, msec);*/

  while (1) {
    manlySilverArm.setPosition(RotationSensor.position(deg), deg);
    if (mSABool) {
      if (RotationSensor.position(deg) < 17.5) {
        // printToConsole("wowie");
        printToConsole("motor " << manlySilverArm.position(deg));
        manlySilverArm.spinTo(22, deg, 880, dps, true);
        printToConsole("after " << RotationSensor.position(deg));
        // belt.spinFor(2, sec);
      }
    }

    if (RotationSensor.position(deg) >= 17) {
        ready = true;
    } else {
      ready = false;
    }

    if (OpticalSensor.isNearObject()) {
      runTime = true;
    }

    if (runTime) {
      time += 20;
    }
    
    if (runTime2) {
      time2 += 20;
    }

    if (ready && time >= 500) {
      spin_for_rev(1, belt, 6);
      printToConsole("spin backwards");
      ready = false;
      runTime = false;
      time2 = 0;
      runTime2 = true;
      time = 0;
    }

    if (time2 >= 500) {
      belt.stop(coast);
      runTime2 = false;
      time2 = 0;
    }

    printToConsole(InertialSensor.heading());

    // printToConsole("time: " << time);
    // printToConsole("time2: " << time2);

    // printToConsole("mSABool " << mSABool);
    // printToConsole("no conditional " << RotationSensor.position(deg)); //} if (false) { //isolating lady brown for testing

    //PID test controls
    /*if (!Controller1.ButtonX.pressing() && !Controller1.ButtonY.pressing()) {
      if (Controller1.ButtonB.pressing()) {
        printToConsole("PID running");
        pid.runPID(distance, 10);
      } else if (Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()) {
        distance = 0;
        printToConsole(distance);
        wait(0.1, sec);
      }  else if (Controller1.ButtonRight.pressing()) {
        distance++;
        printToConsole(distance);
        wait(0.1, sec);
      } else if (Controller1.ButtonLeft.pressing()) {
        distance--;
        printToConsole(distance);
        wait(0.1, sec);
      }
    } else if (Controller1.ButtonX.pressing()){
      if (Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()) {
        pid.kpUpdate(0-pid.kpUpdate(0));
        pid.kpUpdate(0.5);
        printToConsole(pid.kpUpdate(0));
        wait(0.1, sec);
      }  else if (Controller1.ButtonRight.pressing()) {
        pid.kpUpdate(0.1);
        printToConsole(pid.kpUpdate(0));
        wait(0.1, sec);
      } else if (Controller1.ButtonLeft.pressing()) {
        pid.kpUpdate(-0.1);
        printToConsole(pid.kpUpdate(0));
        wait(0.1, sec);
      }
    } else if (Controller1.ButtonY.pressing()){
      if (Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()) {
        pid.kdUpdate(0-pid.kdUpdate(0));
        pid.kdUpdate(0.1);
        printToConsole(pid.kdUpdate(0));
        wait(0.1, sec);
      }  else if (Controller1.ButtonRight.pressing()) {
        pid.kdUpdate(0.1);
        wait(0.01, sec);
        printToConsole(pid.kdUpdate(0));
        wait(0.1, sec);
      } else if (Controller1.ButtonLeft.pressing()) {
        pid.kdUpdate(-0.1);
        wait(0.01, sec);
        printToConsole(pid.kdUpdate(0));
        wait(0.1, sec);
      }
    }*/
  
    // printToConsole(((fabs(Left.position(vex::turns)) + fabs(Right.position(vex::turns))) / 2.0));

    if (Controller1.ButtonUp.pressing()){
      slow = 0.4;
    } else {
      slow = 1;
    }

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based  on feedback from the joysticks.
    Left.spin(forward, (Controller1.Axis3.position()+ Controller1.Axis1.position()) * slow * 0.8, pct);
    Right.spin(forward, (Controller1.Axis3.position() - Controller1.Axis1.position()) * slow * 0.8, pct);

    /*Brain.screen.print(Controller1.Axis3.position());
    Brain.screen.print(Controller1.Axis3.position() + Controller1.Axis1.position(), pct);
    Brain.screen.clear();*/ 
    //Ml.spin(forward, Controller1.Axis3.position() + Controller1.Axis1.position(), pct);
    //Mr.spin(forward, Controller1.Axis3.position() - Controller1.Axis1.position(), pct);
    //Bl.spin(forward, Controller1.Axis3.position() + Controller1.Axis1.position(), pct);
    //Br.spin(forward, Controller1.Axis3.position() - Controller1.Axis1.position(), pct);
    
    

    if (Controller1.ButtonDown.pressing()) {
      //from the screen
      Left.stop(hold);
      Right.stop(hold);
    } else {
      //to the ring
      Left.setStopping(coast);
      Right.setStopping(coast);
    }

    if (Controller1.ButtonR1.pressing()) {
      //to the pen
      belt.spin(forward, 100, pct);
    } else if (Controller1.ButtonR2.pressing()) {
      //to the king
      belt.spin(reverse, 100, pct);
    } else {
      //where's my crown
      belt.stop(brake);
    }
    
    if (Controller1.ButtonA.pressing()) {
      //that's my bling
      if (pressingBool == false) {
        pneumaticsBool = !pneumaticsBool;
        clampPneumatics.set(pneumaticsBool);
        printToConsole(pneumaticsBool);
      }
      pressingBool = true;
    } else {
      //always drama when I ring
      pressingBool = false;
      // printToConsole("Pneumatics set to ");
    }
    // printToConsole("Pneumatics set to " <<pneumaticsBool);


    /*printToConsole("outside checks");
    if (mSABool && RotationSensor.position(deg) > 17.5) {
      printToConsole("brake");
      manlySilverArm.stop(brake);
    }
    if (mSABool && RotationSensor.position(deg) < 17.5) {
        manlySilverArm.spin(forward);
    } else if (mSABool) {
      mSABool = false;
    }*/


    if (Controller1.ButtonL1.pressing()) {
      if (RotationSensor.position(deg) < 17.5) {
          mSABool = true;
          //manlySilverArm.spin(forward,20,pct);
      }  else {
        mSABool = false;
      }
      if (!mSABool) { manlySilverArm.spin(forward, 100, pct); }
    } else if (Controller1.ButtonL2.pressing()) {
      manlySilverArm.spin(reverse, 100, pct);
      if (RotationSensor.position(deg) > 17.5) { mSABool = false; }
    } else {
      if (RotationSensor.position(deg) < 17.5) manlySilverArm.stop(coast);
      else manlySilverArm.stop(hold);
    } 
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
