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
// LeftMotor1           motor         10              
// RightMotor1          motor         1               
// RightMotor2          motor         2               
// LeftMotor2           motor         9               
// RightIntakeMotor     motor         5               
// LeftIntakeMotor      motor         8               
// Controller1          controller                    
// Tilter               motor         3               
// Lift                 motor         6               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
using namespace vex;
/*
vex::motor RightMotor1(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor1(vex::PORT10, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor2(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor2(vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor RightIntakeMotor(vex::PORT5, vex::gearSetting::ratio18_1, true);
vex::motor LeftIntakeMotor(vex::PORT8, vex::gearSetting::ratio18_1, true);
vex::controller Controller1 = vex::controller();
*/
// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

////////////////////
/////////////////////

//If the Robot is slightly off, remove the integral loop for the dirvetrain

////////////////////
////////////////////

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


// PID Settings - these are the variables modified for the PID  
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
//////////////////////////////
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

// Autonomous Settings
int desiredValue = 200; // where you want the robot to go 
int desiredTurnValue = 0;

int error; // Current placement - Where you want to be : Positional Value (Kind of like horizontal Displacement)
int PrevError = 0; // error 20 milliseconds ago 
int derivitive;// error - previous error; and based on that it adjusts the speed
int totalError = 0;// totalerror + error

int turnError; // Current placement - Where you want to be : Positional Value (Kind of like horizontal Displacement)
int turnPrevError = 0; // error 20 milliseconds ago 
int turnDerivitive;// error - previous error; and based on that it adjusts the speed
int turnTotalError = 0;// totalerror + error

bool resetDriveSensors = false;



bool enableDrivePID = true; // allowes the PID to run or stop using boolean varables

int drivePID(){
  while(enableDrivePID) {   // only runs when switch equals to true
    if (resetDriveSensors){
      resetDriveSensors = false;
      LeftMotor1.setPosition(0,degrees);
      RightMotor1.setPosition(0,degrees);
      LeftMotor2.setPosition(0,degrees);
      RightMotor2.setPosition(0,degrees);



    }
  //gets the position of the motors
    int LeftMotor1Position = LeftMotor1.position(degrees);
    int RightMotor1Position = RightMotor1.position(degrees);
    int LeftMotor2Position = LeftMotor2.position(degrees);
    int RightMotor2Position = RightMotor2.position(degrees);

    ////////////////////////////////////////////////////

    // Lateral Movement PID 

    ///////////////////////////////////////////////////

    //Finds the Average of the motors  
    int averagePosition = (LeftMotor1Position + RightMotor1Position+  LeftMotor2Position + RightMotor2Position) / 4;


    //Potential
    error = averagePosition - desiredValue;


    //Derivitive
    derivitive = error - PrevError ; 

    //Integral
    totalError += error;


    double lateralMotorPower = (error * kP + derivitive *kD + totalError *kI) / 12.0; // remove 12.0 if doesnt work

    ////////////////////////////////////////////////////

    // Turning PID  

    ///////////////////////////////////////////////////
    //Finds the Average of the motors  
    int turnDifference = ((LeftMotor1Position +  LeftMotor2Position) - (RightMotor1Position + RightMotor2Position));


    //Potential
    turnError = turnDifference - desiredTurnValue;


    //Derivitive
    turnDerivitive = turnError - turnPrevError ; 

    //Integral
    turnTotalError += turnError;


    double turnMotorPower = (turnError * turnkP + turnDerivitive * turnkD + turnTotalError * turnkI) / 12.0;




    ///////////////////////////////////////////////////
    LeftMotor1.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RightMotor1.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    LeftMotor2.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RightMotor1.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);


    PrevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);
  }


  return 1;
}

//one motor for the tilter. 
//////////////////////////////////////////
/*
int tilterRemap() {
  double CurrentPosition = Tilter.position(degrees);
  double TrayDownPosition =  15.0;    // initial position in degrees
  double TrayFinalPosition = 102.0;   // final position in degrees
  double MotorStartSpeed = 0.0;
  double MotorEndSpeed = 0.0;

  MotorStartSpeed += TrayFinalPosition ; 

  MotorEndSpeed += TrayDownPosition;

 ///////////////////////////////////// 


















return 1 ;
}
*/


void tilterSpeed(double trayFinal){     // degrees of final position    
  double CurrentPosition = Tilter.position(degrees);  // the current position 
  double motorSpeed = 0.0;  // just defining the motor speed


  while (CurrentPosition < trayFinal ){                         // a while loop that converts the motor speed to the current position in degrees 
    double CurrentPosition = Tilter.position(degrees);
    motorSpeed += CurrentPosition;
    Tilter.spin(fwd, motorSpeed, velocityUnits::pct);     // make the motors move based on the new motor speed made
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

  tilterSpeed(100); // calling the fucntion








  vex::task forward(drivePID);


 
  resetDriveSensors = true;
  RightIntakeMotor.setVelocity(100, vex::velocityUnits::pct);
  LeftIntakeMotor.setVelocity(100, vex::velocityUnits::pct);
/////
  RightIntakeMotor.rotateFor(12.35, vex::rotationUnits::rev, false);
  LeftIntakeMotor.rotateFor(-12.35, vex::rotationUnits::rev,false);
  desiredValue = 300;      // move 300 degrees forward
  desiredTurnValue = 0;
  ///////////////////////////////////////////////////////////
  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = 0;
  desiredTurnValue = 150; // turn 150 degrees
  ///////////////////////////////////////////////////////////
  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = 0;
  desiredTurnValue = 150; // turn 150 degrees
  //////////////////////////////////////////////////////////
  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = 250; // move forward 250 degrees
  desiredTurnValue = 0; 
  /////////////////////////////////////////////////
  


  




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
  enableDrivePID = false;
  // settings
  double turnImportance = 0.5;
  while (1) {


    ////// voltage is alot faster, more responsive, and more reliable than rotationa and other values
    // Arcade Drive
    double turnVal = Controller1.Axis1.position(percent);
    double forwardVal = Controller1.Axis3.position(percent);
    //double forwardVal2 = Controller1.Axis3.position(percent);

    double turnVolts = turnVal *0.12;
    double forwardVolts = forwardVal *0.12 *( 1- (std::abs(turnVolts)/12) * turnImportance);
    //double forwardVolts2 = forwardVal *0.12 *( 1- (std::abs(turnVolts)/12) * turnImportance);


    LeftMotor1.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
    RightMotor1.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
    LeftMotor2.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
    RightMotor2.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

/////////////////////////
//Tank Drive
/*     
    double forwardVal = Controller1.Axis3.position(percent);
    double forwardVal2 = Controller1.Axis3.position(percent);

    double forwardVolts = forwardVal *0.12 *( 1- (std::abs(turnVolts)/12) * turnImportance);
    double forwardVolts2 = forwardVal *0.12 *( 1- (std::abs(turnVolts)/12) * turnImportance);

    LeftMotor1.spin(forward, forwardVolts, voltageUnits::volt);
    RightMotor1.spin(forward, forwardVolts2, voltageUnits::volt);
    LeftMotor2.spin(forward, forwardVolts, voltageUnits::volt);
    RightMotor2.spin(forward, forwardVolts2, voltageUnits::volt);
*/

/////////////////////////////////////////////////////////////////////////////////////////////////
//lift control

    bool buttonA = Controller1.ButtonA.pressing();
    bool buttonB = Controller1.ButtonB.pressing();

    if (buttonA){
      Lift.spin(forward,12.0, voltageUnits::volt);
      

    }
    else if (buttonB){
      Lift.spin(forward,-12.0, voltageUnits::volt);
    }
    else {
      Lift.spin(forward,0, voltageUnits::volt);
    }
//////////////////////////////////////////////////////////////////////////////////////
//tilterControl
    bool Tilter1 = Controller1.ButtonR1.pressing();
    bool Tilter2 = Controller1.ButtonR2.pressing();

    if (Tilter1){
      Tilter.spin(forward,12.0, voltageUnits::volt);
      

    }
    else if (Tilter2){
      Tilter.spin(forward,-12.0, voltageUnits::volt);
    }
    else {
      Tilter.spin(forward,0, voltageUnits::volt);
    }
//////////////////////////////////////////////////////////////////////////////////////
//Intake Control
    bool Intake1 = Controller1.ButtonL1.pressing();
    bool Intake2 = Controller1.ButtonL2.pressing();

    if (Intake1){
      LeftIntakeMotor.spin(forward,12.0, voltageUnits::volt);
      RightIntakeMotor.spin(forward,12.0, voltageUnits::volt);
      

    }
    else if (Intake2){
      LeftIntakeMotor.spin(forward,-12.0, voltageUnits::volt);
      RightIntakeMotor.spin(forward,-12.0, voltageUnits::volt);
    }
    else {
      Lift.spin(forward,0, voltageUnits::volt);
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
