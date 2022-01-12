#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller con;
pneumatics mG = pneumatics(Brain.ThreeWirePort.H);
motor lMotor1 = motor(PORT20,ratio6_1,  true);
motor lMotor2 = motor(PORT5,ratio6_1, true);
motor rMotor1 = motor(PORT11,ratio6_1, false);
motor rMotor2 = motor(PORT4,ratio6_1, false);
motor ml1 = motor (PORT3, ratio36_1, true);
motor ml2 = motor(PORT6, true);
motor ml3 = motor(PORT17, false);
motor rl = motor(PORT13, true);
rotation michia = rotation(PORT10);
inertial tom = inertial(PORT7);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}