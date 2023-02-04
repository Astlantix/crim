#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor fr = motor(PORT12, ratio18_1, true);
motor br = motor(PORT15, ratio18_1, true);
motor fl = motor(PORT16, ratio18_1, false);
motor bl = motor(PORT17, ratio18_1, false);
motor spinny = motor(PORT1, ratio18_1, true);
motor flywheelMotorA = motor(PORT18, ratio6_1, false);
motor flywheelMotorB = motor(PORT19, ratio6_1, true);
motor_group flywheel = motor_group(flywheelMotorA, flywheelMotorB);
inertial Inertial = inertial(PORT3);
optical Optical = optical(PORT20);
digital_out shooter = digital_out(Brain.ThreeWirePort.A);
controller gamers = controller(primary);
motor expansion = motor(PORT2, ratio36_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}