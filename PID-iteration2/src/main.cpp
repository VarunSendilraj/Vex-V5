/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VarunSendilraj-109x                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  OlaCompStatesTournament                                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftMotor1           motor         9
// RightMotor1          motor         3
// RightMotor2          motor         1
// LeftMotor2           motor         10
// RightIntakeMotor     motor         12
// LeftIntakeMotor      motor         18
// Controller1          controller
// Tilter               motor         11
// Lift                 motor         20
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
using namespace vex;
// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

// If the Robot is slightly off, remove the integral loop for the dirvetrain

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
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

double kP = 0.3;
double kI = 0.0;
double kD = 0.0;
double turnkP = 0.3;
double turnkI = 0.0;
double turnkD = 0.0;

//Autonomous Settings
int desiredValue = 200;
int desiredTurnValue = 0;

int error; //SensorValue - DesiredValue : Position
int prevError = 0; //Position 20 miliseconds ago
int derivative; // error - prevError : Speed
int totalError = 0; //totalError = totalError + error

int turnError; //SensorValue - DesiredValue : Position
int turnPrevError = 0; //Position 20 miliseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError = 0; //totalError = totalError + error

bool resetDriveSensors = false;

//Variables modified for use
bool enableDrivePID = true;

int drivePID(){
  
  while(enableDrivePID){

    if (resetDriveSensors) {
      resetDriveSensors = false;
      LeftMotor1.setPosition(0,degrees);
      RightMotor1.setPosition(0,degrees);
      LeftMotor2.setPosition(0,degrees);
      RightMotor2.setPosition(0,degrees);
    }


    //Get the position of both motors
    int leftMotorPosition = LeftMotor1.position(degrees);
    int rightMotorPosition = RightMotor1.position(degrees);

    ///////////////////////////////////////////
    //Lateral movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int averagePosition = ((leftMotorPosition )  + (rightMotorPosition))/2;

    //Potential
    error = averagePosition - desiredValue;

    //Derivative
    derivative = error - prevError;

    //Integral
    totalError += error;

    double lateralMotorPower = error * kP + derivative * kD + totalError * kI;
    /////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////
    //Turning movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int turnDifference = ((leftMotorPosition - rightMotorPosition)/2);

    //Potential
    turnError = turnDifference - desiredTurnValue;

    //Derivative
    turnDerivative = turnError - turnPrevError;

    //Integral
    turnTotalError += turnError;

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;
    /////////////////////////////////////////////////////////////////////

    LeftMotor1.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RightMotor1.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }

  return 1;
}


// TilterMacros settings
double tilterkP = 0.5;

int desiredTrayAngle = -500;

int Trayerror = 0; // Dont Touch

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
  vex::task forward(drivePID);
  
  resetDriveSensors = true;
  desiredValue = 200; // move 300 degrees forward
  desiredTurnValue = 300;
  Brain.Screen.print("Lateral PID ");
  Brain.Screen.newLine();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  enableDrivePID = false; // disables Drive PID

  while (true) {
    ////////////////////////////////////////////////////////////////////////////////////////////
    // Tank Drive
    ///////////////////////////////////////////////////////////////////////////////////

    // tank drive settings
    double forwardVal = Controller1.Axis3.position(percent);
    double forwardVal2 = Controller1.Axis2.position(percent);
    // actual tank drive
    // Tank Drive
    double forwardVolts = forwardVal * 0.12;
    double forwardVolts2 = forwardVal2 * 0.12;

    LeftMotor1.spin(forward, forwardVolts, voltageUnits::volt);
    RightMotor1.spin(forward, forwardVolts2, voltageUnits::volt);
    LeftMotor2.spin(forward, forwardVolts, voltageUnits::volt);
    RightMotor2.spin(forward, forwardVolts2, voltageUnits::volt);
 


    /////////////////////////////////////////////////////////////////////////////////////////////////
    // lift control
    ///////////////////////////////////////////////////////////////////////////

    bool buttonUp = Controller1.ButtonR2.pressing();
    bool buttonDown = Controller1.ButtonR1.pressing();

    if (buttonUp) {
      Lift.spin(forward, 8.0, voltageUnits::volt);

    } else if (buttonDown) {
      Lift.spin(forward, -8.0, voltageUnits::volt);
    } else {
      Lift.spin(forward, 0, voltageUnits::volt);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // tilterControl
    /////////////////////////////////////////////////////////////////////////////////////

    bool Tilter1 = Controller1.ButtonB.pressing();
    bool Tilter2 = Controller1.ButtonA.pressing();

    if (Tilter1) {
      Tilter.spin(forward, 7.0, voltageUnits::volt);

    } else if (Tilter2) {
      Tilter.spin(forward, -7.0, voltageUnits::volt);
    } else {
      Tilter.spin(forward, 0, voltageUnits::volt);
    }

    /////////////////////////////////////////////////////////////
    // tiltercontrolerMacros

    if (Controller1.ButtonY.pressing()) {
      error = desiredTrayAngle - Tilter.position(degrees);
      double motorPower = Trayerror * tilterkP;
      Tilter.spin(forward, motorPower, voltageUnits::volt);
    } else {
      Tilter.spin(forward, 0, voltageUnits::volt);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // Intake Control
    bool Intake1 = Controller1.ButtonL1.pressing();
    bool Intake2 = Controller1.ButtonL2.pressing();

    if (Intake1) {
      LeftIntakeMotor.spin(forward, 12.0, voltageUnits::volt);
      RightIntakeMotor.spin(forward, 12.0, voltageUnits::volt);

    } else if (Intake2) {
      LeftIntakeMotor.spin(forward, -12.0, voltageUnits::volt);
      RightIntakeMotor.spin(forward, -12.0, voltageUnits::volt);
    } else {
      LeftIntakeMotor.spin(forward, 0, voltageUnits::volt);
      RightIntakeMotor.spin(forward, 0, voltageUnits::volt);
    }

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
