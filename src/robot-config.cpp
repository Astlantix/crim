#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor rf = motor(PORT12, ratio18_1, true);
motor rb = motor(PORT15, ratio18_1, true);
motor lf = motor(PORT16, ratio18_1, false);
motor lb = motor(PORT17, ratio18_1, false);
motor roller = motor(PORT1, ratio18_1, true);
motor fly_wheelMotorA = motor(PORT18, ratio6_1, false);
motor fly_wheelMotorB = motor(PORT19, ratio6_1, true);
motor_group fly_wheel = motor_group(fly_wheelMotorA, fly_wheelMotorB);
inertial Inertial = inertial(PORT3);
optical Optical = optical(PORT20);
digital_out indexer = digital_out(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);
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