#include "vex.h"
#include "flywheel.hpp"
#include "flywheel.cpp"
#include "robot-config.h"
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	int currTime = 0;
	int finalTime = 0;
	int delta = 0;

	for (size_t i = 0; i < 15; i++)
	{
		startFlywheel();
		currTime = time(nullptr) * 1000;
		flywheelWaitUntilSettled();
		finalTime = time(nullptr) * 1000;
		delta = finalTime - currTime;
		printf("%d\n", delta);
		flywheelStop();
		wait(50000,msec);
	}
	// startFlywheel();
	while (true)
	{
		wait(20, msec);
	}
}