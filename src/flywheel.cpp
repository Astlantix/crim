
#include "robot-config.h"
#include "flywheel.hpp"
#include "vex.h"




const float FLYWHEEL_GAIN = .01;

const int FLYWHEEL_HIGH = 300;

bool flywheelEnabled = false;

bool flywheelSettled = false;

static int flywheelSlewSpeed = 0;

const double FLYWHEEL_kP = .8;
const double FLYWHEEL_kI = 0.1;
const double FLYWHEEL_kD = .1;

double flywheelCurrentVelocity = 0;
double flywheelError = 0;
double flywheelPreviousError = 0;
double flywheelTargetVelocity = 0;
double flywheelOutput = 0;
double flywheelTBH = 0;
double flywheelGain = FLYWHEEL_GAIN;

double flywheelDerivative = 0;
double flywheelIntegral = 0;

int flywheelAccelStep = 4;
int flywheelDeccelStep = 256;

template <typename T>
int sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}

void _flywheelSlew(int target)
{
	int step;

	if (abs(flywheelSlewSpeed) < abs(target))
		step = flywheelAccelStep;
	else
		step = flywheelDeccelStep;

	if (target > flywheelSlewSpeed + step)
		flywheelSlewSpeed += step;
	else if (target < flywheelSlewSpeed - step)
		flywheelSlewSpeed -= step;
	else
		flywheelSlewSpeed = target;

	flywheel.move(flywheelSlewSpeed);
}

void _TBHFlywheel()
{
	flywheelCurrentVelocity = flywheel.get_actual_velocity();
	flywheelError = flywheelTargetVelocity - flywheelCurrentVelocity;
	flywheelOutput += FLYWHEEL_GAIN * flywheelError;
	if (signbit(flywheelError) != signbit(flywheelPreviousError))
	{
		flywheelOutput = 0.5 * (flywheelOutput + flywheelTBH);
		flywheelTBH = flywheelOutput;
		flywheelPreviousError = flywheelError;
	}
	_flywheelSlew(flywheelOutput);

	flywheelSettled = settleUtil.isSettled(flywheelError);
}
void _PIDFLywheel()
{
	flywheelCurrentVelocity = flywheel.get_actual_velocity();
	flywheelError = flywheelTargetVelocity - flywheelCurrentVelocity;
	flywheelIntegral += flywheelError;
	if (flywheelIntegral > 60 / (FLYWHEEL_kI))
	{
		flywheelIntegral = 60 / (FLYWHEEL_kI);
	}
	flywheelDerivative = flywheelError - flywheelPreviousError;
	flywheelPreviousError = flywheelError;
	flywheelOutput = flywheelError * FLYWHEEL_kP + flywheelIntegral * FLYWHEEL_kI + flywheelDerivative * FLYWHEEL_kD;
	_flywheelSlew(flywheelOutput);

	flywheelSettled = settleUtil.isSettled(flywheelError);
}

void flywheelTask(void *parameter)
{

	while (true)
	{

		if (flywheelEnabled)
		{
			_TBHFlywheel();
			// _PIDFLywheel();
		}
		else
		{
			flywheelSettled = false;
			flywheel.move(0);
		}
		// printf("Vel=%.0f Out=%d Settled: %d Error: %.0f P=%.0f I= %.0f D= %.0f\n", flywheel.get_actual_velocity(), flywheelSlewSpeed, flywheelSettled, flywheelError, flywheelError * FLYWHEEL_kP, flywheelIntegral * FLYWHEEL_kI, flywheelDerivative * FLYWHEEL_kD);
		wait(20,msec);
	}
}

void flywheelStop()
{
	flywheelEnabled = false;
}

void _resetVariables()
{
	flywheelSlewSpeed = 0;

	flywheelCurrentVelocity = 0;
	flywheelError = 0;
	flywheelPreviousError = 0;
	flywheelTargetVelocity = 0;
	flywheelOutput = 0;
	flywheelTBH = 0;
}

void startFlywheel()
{
	_resetVariables();
	flywheelEnabled = true;
	flywheelSlewSpeed = 0;
	flywheelTargetVelocity = FLYWHEEL_HIGH;
}

void flywheelWaitUntilSettled()
{
	while (!flywheelSettled)
	{
		wait(20,msec);
	}
}

bool isFlywheelSettled()
{
	return flywheelSettled;
}