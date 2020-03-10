using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftMotor1;
extern motor RightMotor1;
extern motor RightMotor2;
extern motor LeftMotor2;
extern motor RightIntakeMotor;
extern motor LeftIntakeMotor;
extern controller Controller1;
extern motor Tilter;
extern motor Lift;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );