#ifndef _FLYWHEEL_HPP_
#define _FLYWHEEL_HPP_

void flywheelTask(void *);
void flywheelOp(void *);

void startFlywheel();
void flywheelStop();

bool isFlywheelSettled();

void flywheelWaitUntilSettled();
void flywheelPrintInfoToLCD();
#endif