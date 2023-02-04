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
// fr                  motor         12              
// br                   motor         15              
// fl                   motor         16              
// bl                   motor         17              
// spinny               motor         1               
// flywheel            motor_group   18, 19          
// Inertial             inertial      3               
// Optical              optical       20              
// shooter              digital_out   A               
// gamers          controller                    
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
      (gamers.Axis3.position() * -y) + (gamers.Axis4.position() * x);
  double leftspeed =
      (gamers.Axis3.position() * -y) - (gamers.Axis4.position() * x);
  fl.spin(forward, leftspeed, percent);
  bl.spin(forward, leftspeed, percent);
  fr.spin(forward, rightspeed, percent);
  br.spin(forward, rightspeed, percent);
}

// ..........................................................................
// auton functions
// ..........................................................................
void setV(double x) {
  fl.setVelocity(x, percent);
  bl.setVelocity(x, percent);
  fr.setVelocity(x, percent);
  br.setVelocity(x, percent);
}
void setcoast() {
  fl.setStopping(coast);
  bl.setStopping(coast);
  fr.setStopping(coast);
  br.setStopping(coast);
}
void gofor(double x, double y, double z) {
  setV(y);
  fl.spinFor(forward, -x, degrees, false);
  bl.spinFor(forward, -x, degrees, false);
  fr.spinFor(forward, -x, degrees, false);
  br.spinFor(forward, -x, degrees);
  wait(z, msec);
}
void goback(double x, double y, double z) {
  setV(y);
  fl.spinFor(reverse, -x, degrees, false);
  bl.spinFor(reverse, -x, degrees, false);
  fr.spinFor(reverse, -x, degrees, false);
  br.spinFor(reverse, -x, degrees);
  wait(z, msec);
}
void gobackangled(double x, double y, double z) {
  fr.spin(reverse, -x, percent);
  bl.spin(reverse, -x, percent);
  fr.spin(reverse, -y, percent);
  br.spin(reverse, -y, percent);
  wait(z, msec);
  fl.stop();
  bl.stop();
  fr.stop();
  br.stop();
}
void turnright(double x, double y, double z) {
  setV(y);
  fl.spinFor(forward, -x, degrees, false);
  bl.spinFor(forward, -x, degrees, false);
  fr.spinFor(reverse, -x, degrees, false);
  br.spinFor(reverse, -x, degrees);
  wait(z, msec);
}
void turnleft(double x, double y, double z) {
  setV(y);
  fl.spinFor(reverse, -x, degrees, false);
  bl.spinFor(reverse, -x, degrees, false);
  fr.spinFor(forward, -x, degrees, false);
  br.spinFor(forward, -x, degrees);
  wait(z, msec);
}
void printing() {
  gamers.Screen.clearScreen();
  gamers.Screen.setCursor(1, 1);
  if (autoslct == 1) {
    gamers.Screen.print("sleeping");
  }
  if (autoslct == 2) {
    gamers.Screen.print("lg+r Left ");
  }
  if (autoslct == 3) {
    gamers.Screen.print("lg+r Right");
  }
  if (autoslct == 4) {
    gamers.Screen.print("lg+r+hg Left");
  }
  if (autoslct == 5) {
    gamers.Screen.print("lg+r+hg Right");
  }
  if (autoslct == 6) {
    gamers.Screen.print("pl+r+hg Right");
  }
  if (autoslct == 7) {
    gamers.Screen.print("pl+r+hg Left(dont use)");
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
  shooter.set(false);
  flywheel.setVelocity(x, percent);
  wait(100, msec);
  shooter.set(true);
  flywheel.setVelocity(z, percent);
  wait(y, msec);
}

// low goal and roller right
void lgrRight(bool x) {
  setcoast();
  // shooting 2 preloads into the low goal
  if (x) {
    flywheel.spin(forward, lowgoal, percent);
    wait(2500, msec);
    shoot(500, lowgoal, lowgoal);
    shoot(500, lowgoal, lowgoal);
  }
  flywheel.stop(coast);
  // moving towards the roller and rolling
  gofor(600, 50, 0);
  turnright(270, 20, 0);
  spinny.spin(reverse, 100, percent);
  gofor(105, 20, 70);
  spinny.stop();
  goback(125, 50, 0);
}
// low goal and roller left
void lgrLeft(double x) {
  setcoast();
  // move towards the roller and roll
  spinny.spin(reverse, 100, percent);
  gofor(48, 30, 70);
  spinny.stop();
  // moving away from roller and shoot 2 preloads
  goback(70, 30, 0);
  turnleft(360, 30, 0);
  if (x) {
    flywheel.spin(forward, shortdist, percent);
    wait(1000, msec);
    shoot(500, shortdist, shortdist);
    shoot(500, shortdist, shortdist);
    flywheel.stop(coast);
  }
}


// low goal and roller and high goal right
void lgrhgRight() {
  setcoast();
  lgrRight(false);
  turnright(243, 50, 0);
  spinny.spin(forward, 100, percent);
  gofor(145, 100, 0);
  turnright(168, 30, 0);
  flywheel.spin(forward, longdist + 3, percent);
  gofor(1500, 30, 0);
  turnleft(232.5, 50, 1000);
  spinny.stop();
  waitUntil(flywheel.velocity(percent) > 66);
  shoot(500, 65, longdist);
  waitUntil(flywheel.velocity(percent) > 66);
  shoot(300, 100, longdist);
  waitUntil(flywheel.velocity(percent) > 66);
  shoot(500, 100, longdist);
  spinny.stop();
  flywheel.stop(coast);
} // low goal and roller and high goal left
void lgrhgLeft() {
  setcoast();
  double ldist = 69.5;
  lgrLeft(false);
  turnright(408, 30, 100);
  spinny.spin(forward, 100, percent);
  gofor(550, 50, 250);
  flywheel.spin(forward, ldist, percent);
  gofor(1200 - 550, 10, 0);
  turnright(275, 20, 0);
  waitUntil(flywheel.velocity(percent) > 67);
  shoot(500, 70, ldist);
  waitUntil(flywheel.velocity(percent) > 67);
  shoot(300, 100, ldist);
  waitUntil(flywheel.velocity(percent) > 67);
  shoot(300, 100, ldist);
  flywheel.stop(coast);
  spinny.stop();
}
//good final right side
void plrhgRight() {
  setcoast();
  lgrRight(false);
  turnright(387, 50, 0);
  spinny.spin(forward, 100, percent);
  gofor(600, 50, 500);
  flywheel.spin(forward, 68, percent);
  turnleft(283, 50, 0);
  waitUntil(flywheel.velocity(percent) > 67);
  shoot(500, 67, longdist);
  waitUntil(flywheel.velocity(percent) > 67);
  shoot(300, 100, longdist);
  waitUntil(flywheel.velocity(percent) > 67);
  shoot(500, 100, longdist);
  spinny.stop();
  flywheel.stop(coast);
}
//good final left side DONT TEST
void plrhgLeft() {
  setcoast();
  lgrLeft(false);
  spinny.spin(forward, 100, percent);
  gofor(601, 80, 500);
  turnright(280,30,500);
  flywheel.spin(forward,75,percent);
  waitUntil(flywheel.velocity(percent) >= 70);
  shoot(500, 70, 68.5);
  waitUntil(flywheel.velocity(percent) >= 70);
  shoot(300, 100, 69);
  waitUntil(flywheel.velocity(percent) >= 69);
  shoot(300, 100, 68.5);
  flywheel.stop(coast);
  spinny.stop();
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
  shooter.set(true);
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
  shooter.set(true);
  gamers.Screen.print("sleeping");
  gamers.ButtonLeft.pressed(autominus);
  gamers.ButtonRight.pressed(autoplus);
  while (a) {

    if (gamers.ButtonA.pressing()) {
      a = false;
    }
  }
  wait(1, sec);
  while (!a) {
    // ..........................................................................
    // printing temp/speed
    // ..........................................................................
    int df = flywheel.velocity(rpm);
    gamers.Screen.clearScreen();
    gamers.Screen.setCursor(1, 1);
    gamers.Screen.print(df);
    // ..........................................................................
    // drivetrain
    // ..........................................................................
    // changing between modes
    if (gamers.ButtonR1.pressing()) {
      dtslowmo = true;
    } else if (gamers.ButtonR2.pressing()) {
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
    if (gamers.ButtonUp.pressing()) {
      spinny.spin(forward, 100, percent);
    } else if (gamers.ButtonDown.pressing()) {
      spinny.spin(reverse, 100, percent);
    } else if (gamers.ButtonLeft.pressing()) {
      spinny.stop(coast);
    }

    // ..........................................................................
    // shooter/indexer
    // ..........................................................................
    // shooter

    if (toggle) {
      flywheel.spin(forward, shortdist, percent);
    } else {
      flywheel.stop(coast);
    }

    if (gamers.ButtonA.pressing()) {
      if (!latch) {
        toggle = !toggle;
        latch = true;
      }
    } else {
      latch = false;
    }

    // shooter
    if (gamers.ButtonL1.pressing()) {
      shooter.set(false);
    } else {
      shooter.set(true);
    }

    // ..........................................................................
    // expansion
    // ..........................................................................
    if (gamers.ButtonA.pressing() && gamers.ButtonB.pressing() &&
        gamers.ButtonX.pressing() && gamers.ButtonY.pressing()) {
      expansion.spin(forward, 50, percent);
      gamers.Screen.clearScreen();
      gamers.Screen.setCursor(1, 1);
      gamers.Screen.print("expansion fired");
      flywheel.stop(coast);
    }
    else if (gamers.ButtonUp.pressing() && gamers.ButtonDown.pressing() &&
        gamers.ButtonRight.pressing() && gamers.ButtonLeft.pressing()) {
      expansion.spin(reverse, 50, percent);
      spinny.stop(coast);
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
