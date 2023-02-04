/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// rf                   motor         12              
// rb                   motor         15              
// lf                   motor         16              
// lb                   motor         17              
// roller               motor         1               
// fly_wheel            motor_group   18, 19          
// Inertial             inertial      3               
// Optical              optical       20              
// indexer              digital_out   A               
// Controller1          controller                    
// expansion            motor         2               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
bool a = true;
// ..........................................................................
// drivetrain variables
// ..........................................................................
bool dtslowmo = false;
double inches2degrees(double inches) { return 23 * inches; }
// every 23 degrees is an inch 550 is 2 feet
double degrees2turns(double degrees) { return 2.8 * degrees; }
// every 2.8 degrees is a degree 250 is 90 degrees
// ..........................................................................
// fly wheel variables
// ..........................................................................
bool toggle = false;
bool latch = false;
double lowgoal = 30;
double shortdist = 60;

double longdist = 67;
// ..........................................................................
// auton variables
// ..........................................................................
int numofautons = 7;
int autoslct = 1;
// drive code
void dtcode(double y, double x) {
  double rightspeed =
      (Controller1.Axis3.position() * -y) + (Controller1.Axis4.position() * x);
  double leftspeed =
      (Controller1.Axis3.position() * -y) - (Controller1.Axis4.position() * x);
  lf.spin(forward, leftspeed, percent);
  lb.spin(forward, leftspeed, percent);
  rf.spin(forward, rightspeed, percent);
  rb.spin(forward, rightspeed, percent);
}

// ..........................................................................
// auton functions
// ..........................................................................
void setV(double x) {
  lf.setVelocity(x, percent);
  lb.setVelocity(x, percent);
  rf.setVelocity(x, percent);
  rb.setVelocity(x, percent);
}
void setcoast() {
  lf.setStopping(coast);
  lb.setStopping(coast);
  rf.setStopping(coast);
  rb.setStopping(coast);
}
void gofor(double x, double y, double z) {
  setV(y);
  lf.spinFor(forward, -x, degrees, false);
  lb.spinFor(forward, -x, degrees, false);
  rf.spinFor(forward, -x, degrees, false);
  rb.spinFor(forward, -x, degrees);
  wait(z, msec);
}
void goback(double x, double y, double z) {
  setV(y);
  lf.spinFor(reverse, -x, degrees, false);
  lb.spinFor(reverse, -x, degrees, false);
  rf.spinFor(reverse, -x, degrees, false);
  rb.spinFor(reverse, -x, degrees);
  wait(z, msec);
}
void gobackangled(double x, double y, double z) {
  lf.spin(reverse, -x, percent);
  lb.spin(reverse, -x, percent);
  rf.spin(reverse, -y, percent);
  rb.spin(reverse, -y, percent);
  wait(z, msec);
  lf.stop();
  lb.stop();
  rf.stop();
  rb.stop();
}
void turnright(double x, double y, double z) {
  setV(y);
  lf.spinFor(forward, -x, degrees, false);
  lb.spinFor(forward, -x, degrees, false);
  rf.spinFor(reverse, -x, degrees, false);
  rb.spinFor(reverse, -x, degrees);
  wait(z, msec);
}
void turnleft(double x, double y, double z) {
  setV(y);
  lf.spinFor(reverse, -x, degrees, false);
  lb.spinFor(reverse, -x, degrees, false);
  rf.spinFor(forward, -x, degrees, false);
  rb.spinFor(forward, -x, degrees);
  wait(z, msec);
}
void printing() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  if (autoslct == 1) {
    Controller1.Screen.print("sleeping");
  }
  if (autoslct == 2) {
    Controller1.Screen.print("lg+r Left ");
  }
  if (autoslct == 3) {
    Controller1.Screen.print("lg+r Right");
  }
  if (autoslct == 4) {
    Controller1.Screen.print("lg+r+hg Left");
  }
  if (autoslct == 5) {
    Controller1.Screen.print("lg+r+hg Right");
  }
  if (autoslct == 6) {
    Controller1.Screen.print("pl+r+hg Right");
  }
  if (autoslct == 7) {
    Controller1.Screen.print("pl+r+hg Left(dont use)");
  }
}
void autoplus() {
  autoslct += 1;
  if (autoslct > numofautons)
    autoslct = 1;
  printing();
}
void autominus() {
  autoslct -= 1;
  if (autoslct < 1)
    autoslct = numofautons;
  printing();
}

void sleeping() { wait(15, seconds); }

void shoot(double y, double x, double z) {
  indexer.set(false);
  fly_wheel.setVelocity(x, percent);
  wait(100, msec);
  indexer.set(true);
  fly_wheel.setVelocity(z, percent);
  wait(y, msec);
}

// low goal and roller right
void lgrRight(bool x) {
  setcoast();
  // shooting 2 preloads into the low goal
  if (x) {
    fly_wheel.spin(forward, lowgoal, percent);
    wait(2500, msec);
    shoot(500, lowgoal, lowgoal);
    shoot(500, lowgoal, lowgoal);
  }
  fly_wheel.stop(coast);
  // moving towards the roller and rolling
  gofor(600, 50, 0);
  turnright(270, 20, 0);
  roller.spin(reverse, 100, percent);
  gofor(105, 20, 70);
  roller.stop();
  goback(125, 50, 0);
}
// low goal and roller left
void lgrLeft(double x) {
  setcoast();
  // move towards the roller and roll
  roller.spin(reverse, 100, percent);
  gofor(48, 30, 70);
  roller.stop();
  // moving away from roller and shoot 2 preloads
  goback(70, 30, 0);
  turnleft(360, 30, 0);
  if (x) {
    fly_wheel.spin(forward, shortdist, percent);
    wait(1000, msec);
    shoot(500, shortdist, shortdist);
    shoot(500, shortdist, shortdist);
    fly_wheel.stop(coast);
  }
}


// low goal and roller and high goal right
void lgrhgRight() {
  setcoast();
  lgrRight(false);
  turnright(243, 50, 0);
  roller.spin(forward, 100, percent);
  gofor(145, 100, 0);
  turnright(168, 30, 0);
  fly_wheel.spin(forward, longdist + 3, percent);
  gofor(1500, 30, 0);
  turnleft(232.5, 50, 1000);
  roller.stop();
  waitUntil(fly_wheel.velocity(percent) > 66);
  shoot(500, 65, longdist);
  waitUntil(fly_wheel.velocity(percent) > 66);
  shoot(300, 100, longdist);
  waitUntil(fly_wheel.velocity(percent) > 66);
  shoot(500, 100, longdist);
  roller.stop();
  fly_wheel.stop(coast);
} // low goal and roller and high goal left
void lgrhgLeft() {
  setcoast();
  double ldist = 69.5;
  lgrLeft(false);
  turnright(408, 30, 100);
  roller.spin(forward, 100, percent);
  gofor(550, 50, 250);
  fly_wheel.spin(forward, ldist, percent);
  gofor(1200 - 550, 10, 0);
  turnright(275, 20, 0);
  waitUntil(fly_wheel.velocity(percent) > 67);
  shoot(500, 70, ldist);
  waitUntil(fly_wheel.velocity(percent) > 67);
  shoot(300, 100, ldist);
  waitUntil(fly_wheel.velocity(percent) > 67);
  shoot(300, 100, ldist);
  fly_wheel.stop(coast);
  roller.stop();
}
//good final right side
void plrhgRight() {
  setcoast();
  lgrRight(false);
  turnright(387, 50, 0);
  roller.spin(forward, 100, percent);
  gofor(600, 50, 500);
  fly_wheel.spin(forward, 68, percent);
  turnleft(283, 50, 0);
  waitUntil(fly_wheel.velocity(percent) > 67);
  shoot(500, 67, longdist);
  waitUntil(fly_wheel.velocity(percent) > 67);
  shoot(300, 100, longdist);
  waitUntil(fly_wheel.velocity(percent) > 67);
  shoot(500, 100, longdist);
  roller.stop();
  fly_wheel.stop(coast);
}
//good final left side DONT TEST
void plrhgLeft() {
  setcoast();
  lgrLeft(false);
  roller.spin(forward, 100, percent);
  gofor(601, 80, 500);
  turnright(280,30,500);
  fly_wheel.spin(forward,75,percent);
  waitUntil(fly_wheel.velocity(percent) >= 70);
  shoot(500, 70, 68.5);
  waitUntil(fly_wheel.velocity(percent) >= 70);
  shoot(300, 100, 69);
  waitUntil(fly_wheel.velocity(percent) >= 69);
  shoot(300, 100, 68.5);
  fly_wheel.stop(coast);
  roller.stop();
}


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

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  setcoast();
  indexer.set(true);
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

void autonomous(void) {
  if (autoslct == 1) {
    sleeping();
  }
  if (autoslct == 2) {
    lgrLeft(true);
  }
  if (autoslct == 3) {
    lgrRight(true);
  }
  if (autoslct == 4) {
    lgrhgLeft();
  }
  if (autoslct == 5) {
    lgrhgRight();
  }
  if (autoslct == 6) {
    plrhgRight();
  }
  if (autoslct == 7) {
    plrhgLeft();
  }
  a = false;
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

void usercontrol(void) {
  indexer.set(true);
  Controller1.Screen.print("sleeping");
  Controller1.ButtonLeft.pressed(autominus);
  Controller1.ButtonRight.pressed(autoplus);
  while (a) {

    if (Controller1.ButtonA.pressing()) {
      a = false;
    }
  }
  wait(1, sec);
  while (!a) {
    // ..........................................................................
    // printing temp/speed
    // ..........................................................................
    int df = fly_wheel.velocity(rpm);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(df);
    // ..........................................................................
    // drivetrain
    // ..........................................................................
    // changing between modes
    if (Controller1.ButtonR1.pressing()) {
      dtslowmo = true;
    } else if (Controller1.ButtonR2.pressing()) {
      dtslowmo = false;
    }
    // actual dt code
    if (dtslowmo) {
      dtcode(0.1, 0.1);
    }

    else {
      dtcode(1, 0.70);
    }

    // ..........................................................................
    // intake/roller
    // ..........................................................................
    if (Controller1.ButtonUp.pressing()) {
      roller.spin(forward, 100, percent);
    } else if (Controller1.ButtonDown.pressing()) {
      roller.spin(reverse, 100, percent);
    } else if (Controller1.ButtonLeft.pressing()) {
      roller.stop(coast);
    }

    // ..........................................................................
    // shooter/indexer
    // ..........................................................................
    // shooter

    if (toggle) {
      fly_wheel.spin(forward, shortdist, percent);
    } else {
      fly_wheel.stop(coast);
    }

    if (Controller1.ButtonA.pressing()) {
      if (!latch) {
        toggle = !toggle;
        latch = true;
      }
    } else {
      latch = false;
    }

    // indexer
    if (Controller1.ButtonL1.pressing()) {
      indexer.set(false);
    } else {
      indexer.set(true);
    }

    // ..........................................................................
    // expansion
    // ..........................................................................
    if (Controller1.ButtonA.pressing() && Controller1.ButtonB.pressing() &&
        Controller1.ButtonX.pressing() && Controller1.ButtonY.pressing()) {
      expansion.spin(forward, 50, percent);
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("expansion fired");
      fly_wheel.stop(coast);
    }
    else if (Controller1.ButtonUp.pressing() && Controller1.ButtonDown.pressing() &&
        Controller1.ButtonRight.pressing() && Controller1.ButtonLeft.pressing()) {
      expansion.spin(reverse, 50, percent);
      roller.stop(coast);
    }
     else {
      expansion.stop();
    }
    wait(20, msec);

  } // end of while true
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
