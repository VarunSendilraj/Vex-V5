/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\kavin_h3kinzq                                    */
/*    Created:      Tue Oct 01 2019                                           */
/*    Description:  Ola 10/05/2019 Competition                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "math.h" //Include math.h in order to gain access to math functions like PI.
using namespace vex;
vex::motor RightDriveMotor(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor LeftDriveMotor(vex::PORT10, vex::gearSetting::ratio6_1, true);
vex::motor RightTilterMotor(vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor LeftTilterMotor(vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor RightLiftMotor(vex::PORT2, vex::gearSetting::ratio6_1, true);
vex::motor LeftLiftMotor(vex::PORT9, vex::gearSetting::ratio6_1,true);
vex::motor RightIntakeMotor(vex::PORT12, vex::gearSetting::ratio18_1, true);
vex::motor LeftIntakeMotor(vex::PORT19, vex::gearSetting::ratio18_1, true);
vex::controller Controller1 = vex::controller();
 
// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
// A global instance of vex::competition
vex::competition Competition;
 
// define your global instances of motors and other devices here
 
 
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
 
void pre_auton( void ) {
 // All activities that occur before the competition starts
 // Example: clearing encoders, setting servo positions, ...
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
 
void autonomous( void ) {
   //LeftDriveMotor.rotateFor(3.82, rotationUnits::rev, 200,velocityUnits::rpm, false);
   //RightDriveMotor.rotateFor(3.82, rotationUnits::fwd, 200,velocityUnits::rpm, false);
   //vex::rotationUnits rotations = vex::rotationUnits::rev;
 
   //int howmany = 6;
   //LeftDriveMotor.startRotateFor(howmany, rotations);
   //RightDriveMotor.rotateFor(howmany, rotations);
 
//CODE BELOW BASED ON DISTANCE
   double wheelDiameterCM  = 10.16; //wheelDiameter is the measurement of a wheel from edge to edge in centimeters.
   double travelTargetCM = 20; //travelTarget will define how far we want the robot to move in centimeters.
 
   double circumference = wheelDiameterCM * M_PI;
   double degreesToRotate = (360 * travelTargetCM) / circumference;
 
   RightDriveMotor.setVelocity(50, vex::velocityUnits::pct);
   LeftDriveMotor.setVelocity(50, vex::velocityUnits::pct);
 
   LeftDriveMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg, false); //This command must be non blocking.
   RightDriveMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg);
 
// CODE BELOW IS BASED ON REVOLUTIONS
   
   //LeftDriveMotor.setVelocity(50, vex::velocityUnits::pct);
   //RightDriveMotor.setVelocity(50, vex::velocityUnits::pct);
 
   //LeftDriveMotor.rotateFor(1, vex::rotationUnits::rev, false);
   //RightDriveMotor.rotateFor(1, vex::rotationUnits::rev);
 
 
 
 
 
 
 
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
 
void usercontrol( void ) {
 // User control code here, inside the loop
 while (true) {
  int RightTilterMotorPCT = 40;
  int LeftTilterMotorPCT = 40;
  int RightLiftMotorPCT = 50;
  int LeftLiftMotorPCT = 50;
  int RightIntakeMotorPCT = 100;
  int LeftIntakeMotorPCT = 100;
 
   //Base
  
    RightDriveMotor.spin(vex::directionType::rev, (Controller1.Axis2.value()), vex::velocityUnits::pct); //(Axis3+Axis4)/2;
    LeftDriveMotor.spin(vex::directionType::rev, (Controller1.Axis3.value()), vex::velocityUnits::pct);//(Axis3-Axis4)/2;
   /*
     RightDriveMotor.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis4.value()), vex::velocityUnits::pct); //(Axis3+Axis4)/2;
    LeftDriveMotor.spin(vex::directionType::rev, (Controller1.Axis3.value() - Controller1.Axis4.value()), vex::velocityUnits::pct);//(Axis3-Axis4)/2;
   */
   //RightTilter
    if(Controller1.ButtonX.pressing()) {
           RightTilterMotor.spin(vex::directionType::fwd, RightTilterMotorPCT, vex::velocityUnits::pct);
       }
       else if(Controller1.ButtonY.pressing()) {
           RightTilterMotor.spin(vex::directionType::rev, RightTilterMotorPCT, vex::velocityUnits::pct);
       }
       else {
           RightTilterMotor.stop(vex::brakeType::coast);
       }
 
   //LeftTilter
    if(Controller1.ButtonX.pressing()) {
           LeftTilterMotor.spin(vex::directionType::rev, LeftTilterMotorPCT, vex::velocityUnits::pct);
       }
       else if(Controller1.ButtonY.pressing()) {
           LeftTilterMotor.spin(vex::directionType::fwd, LeftTilterMotorPCT, vex::velocityUnits::pct);
       }
       else {
           LeftTilterMotor.stop(vex::brakeType::coast);
       }
   //RightLift
    if(Controller1.ButtonR1.pressing()) {
           RightLiftMotor.spin(vex::directionType::rev, RightLiftMotorPCT, vex::velocityUnits::pct);
       }
       else if(Controller1.ButtonR2.pressing()) {
           RightLiftMotor.spin(vex::directionType::fwd, RightLiftMotorPCT, vex::velocityUnits::pct);
       }
       else {
           RightLiftMotor.stop(vex::brakeType::coast);
       }
   //LeftLift
    if(Controller1.ButtonR1.pressing()) {
           LeftLiftMotor.spin(vex::directionType::fwd, LeftLiftMotorPCT, vex::velocityUnits::pct);
       }
       else if(Controller1.ButtonR2.pressing()) {
           LeftLiftMotor.spin(vex::directionType::rev, LeftLiftMotorPCT, vex::velocityUnits::pct);
       }
       else {
           LeftLiftMotor.stop(vex::brakeType::coast);
       }
       //RightIntake
    if(Controller1.ButtonL1.pressing()) {
           RightIntakeMotor.spin(vex::directionType::fwd, RightIntakeMotorPCT, vex::velocityUnits::pct);
       }
       else if(Controller1.ButtonL2.pressing()) {
           RightIntakeMotor.spin(vex::directionType::rev, RightIntakeMotorPCT, vex::velocityUnits::pct);
       }
       else {
          RightIntakeMotor.stop(vex::brakeType::coast);
       }
   //LeftIntake
    if(Controller1.ButtonL1.pressing()) {
           LeftIntakeMotor.spin(vex::directionType::rev, LeftIntakeMotorPCT, vex::velocityUnits::pct);
       }
       else if(Controller1.ButtonL2.pressing()) {
           LeftIntakeMotor.spin(vex::directionType::fwd, LeftIntakeMotorPCT, vex::velocityUnits::pct);
       }
       else {
           LeftIntakeMotor.stop(vex::brakeType::coast);
       }
 
       }
 
   vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources.
 }
 
 
//
// Main will set up the competition functions and callbacks.
//
int main() {
   //Set up callbacks for autonomous and driver control periods.
   Competition.autonomous( autonomous );
   Competition.drivercontrol( usercontrol );
  
   //Run the pre-autonomous function.
   pre_auton();
     
   //Prevent main from exiting with an infinite loop.                       
   while(1) {
     vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
   }   
     
}
 

