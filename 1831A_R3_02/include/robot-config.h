#include "vex.h"


using namespace vex;

extern brain Brain;

// VEXcode devices
extern smartdrive chassis;
extern motor_group Intake;
extern digital_out Clamp;
extern controller Controller1;
extern motor leftMotorA;
extern motor leftMotorB;
extern motor leftMotorC;
extern motor_group RightDriveSmart;
extern motor rightMotorA;
extern motor rightMotorB;
extern motor rightMotorC;
extern motor_group LeftDriveSmart;
extern motor IntakeMotorA;
extern motor IntakeMotorB;
extern motor_group Intake;
extern inertial Inertial;
extern distance Distance1;
extern distance Distance2; 


/**
 
Used to initialize code/tasks/devices added using tools in VEXcode Pro.
This should be called at the start of your int main function.
*/
void  vexcodeInit( void );