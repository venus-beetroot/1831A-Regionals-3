/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Haoran Fang                                               */
/*    Created:      Sep 06, 2024                                              */
/*    Description:  Regionals 2 Competition                                   */
/*    Template:     VEX Competition Template                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    20, 19, 17, 16  
// Intake               motor_group   9, 10           
// Clamp                digital_out   H               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "robot-config.h"
#include <cmath>
#include <cstdlib>

using namespace vex;

// A global instance of competition
competition Competition;

void turnToHeading(double targetHeading);
void clampGoal();


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/



void pre_auton(void) 
{ 
  chassis.setTurnVelocity(100, percent);
  chassis.setDriveVelocity(100, percent);
  Clamp.set(false);
  Inertial.calibrate();
  Inertial.setHeading(0, degrees);
  Intake.setMaxTorque(100,percent);
  Intake.setVelocity(100,percent);
  

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
void autonomous(void) 
{
  
  chassis.setDriveVelocity(50, percent);
  Intake.setVelocity(100, percent);
  Clamp.set(false);
  wait(0.5, seconds);
  chassis.driveFor(reverse, 36, inches, true);
  
  wait(0.5, seconds);
  

// Clamping and outtaking ring
  clampGoal();
  // IntakeMotorA.spin(reverse, 100, percent);
  // IntakeMotorB.spin(forward, 100, percent);
  Intake.spin(reverse);
  wait(5, seconds);
  Intake.stop();
  Clamp.set(false);

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

void auton_skills() 
{
  chassis.setDriveVelocity(75, percent);
  chassis.setTurnVelocity(40, percent);
  Intake.setVelocity(100, percent);


  Clamp.set(false);
  wait(500, msec);
  chassis.driveFor(reverse, 18, inches, true);
  clampGoal();
  wait(500, msec);
  chassis.setDriveVelocity(75, percent);
  Intake.spin(reverse);
  wait(3, sec);
  Intake.stop();
  chassis.turnToHeading(90, degrees);
  wait(500, msec);
  chassis.driveFor(reverse, 24, inches, true);
  wait(500, msec);
  chassis.turnToHeading(135, degrees);
  wait(500, msec);
  chassis.driveFor(reverse, 34.8822509939, inches, true);
  Clamp.set(false);
  wait(500, msec);
  chassis.setDriveVelocity(30, percent);
  chassis.driveFor(reverse, 5, distanceUnits::cm);
}






void usercontrol(void) 
{
  vexcodeInit();
  // User control code here, inside the loop
  while (1) 
  {
    chassis.setTurnVelocity(100, percent);
    
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

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
  pre_auton();
  Competition.autonomous(auton_skills);
  Competition.autonomous(autonomous);

  //interswitch auton_skills and autonomous based on whether doing tournament matches or programming skills

  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  

  // Prevent main from exiting with an infinite loop.
  while (true) 
  {
    wait(100, msec);
  }
}

void turnToHeading(double targetHeading) 
{
  // Set initial turn velocity (adjust as needed)
  chassis.setTurnVelocity(50, percent);

  // Loop until heading error is within tolerance
  while (std::abs(targetHeading - Inertial.heading()) > 3) 
  {  // Tolerance of 3 degrees
    double headingError = targetHeading - Inertial.heading();
    
    // Normalize heading error
    if (headingError > 180) 
    {
      headingError -= 360;
    } 
    else if (headingError < -180) 
    {
      headingError += 360;
    }

    // Adjust turn velocity based on heading error
    chassis.setTurnVelocity(std::abs(headingError) * 0.5, percent); // Linearly adjust velocity

    // Turn the drivetrain
    if (headingError > 0) 
    {
      chassis.turn(right);
    } 
    else 
    {
      chassis.turn(left);
    }

    // Short wait (adjust as needed)
    wait(0.05, sec);
  }

  // Stop the drivetrain
  chassis.stop();
}



void clampGoal()
{
  double valueD1 = Distance1.objectDistance(distanceUnits::cm);
  double valueD2 = Distance2.objectDistance(distanceUnits::cm);


  if(valueD1 <= 15 && valueD2 <= 15)
  {
    chassis.setDriveVelocity(15, percent);
    chassis.driveFor(reverse, 7.5, distanceUnits::cm);
    wait(0.5, sec);
  }
  Clamp.set(true);
  wait(0.3, sec);
  chassis.setDriveVelocity(100, percent);
}






