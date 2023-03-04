#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor fr = motor(PORT12, ratio18_1, true);
motor br = motor(PORT15, ratio18_1, true);
motor fl = motor(PORT7, ratio18_1, true);
motor bl = motor(PORT10, ratio18_1, true);
motor ml = motor(PORT18, ratio18_1, true);
motor mr = motor(PORT19, ratio18_1, true);
motor spinny = motor(PORT1, ratio18_1, true);
motor flywheel = motor(PORT1, ratio18_1, false);
inertial Inertial = inertial(PORT3);
optical Optical = optical(PORT20);
digital_out shooter = digital_out(Brain.ThreeWirePort.A);
controller gamers = controller(primary);
digital_out expansion = digital_out(Brain.ThreeWirePort.B);

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