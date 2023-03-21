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
// fr                   motor         4             
// mr                   motor         11
// br                   motor         2             
// fl                   motor         7
// ml                   motor         18       
// bl                   motor         10        
// spinny               motor         21               
// flywheel             motor         1          
// Inertial             inertial      3               
// Optical              optical       20              
// shooter              digital_out   A               
// gamers               controller                    
// expansion            digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"


using namespace vex;

// A global instance of competition
competition Competition;
bool a = true;




//..........................................................................
// ODOMETRY
//..........................................................................


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

double longdist = 67 ;
// ..........................................................................
// auton variables
// ..........................................................................
int numofautons = 5;
int autoslct = 1;

// my goofy flywheel pid
/*
void flypid(double target) {
  double error = target - flywheel.velocity(percent);
  double kP = 0.1;
  double kI = 0.0001;
  double kD = 0.0001;
  double integral = 0;
  double derivative = 0;
  double lastError = 0;
  double power = 0;
  while (true) {
    error = target - flywheel.velocity(percent);
    integral = integral + error;
    derivative = error - lastError;
    power = (error * kP) + (integral * kI) + (derivative * kD);
    flywheel.spin(forward, power, percent);
    lastError = error;
    wait(20, msec);
  }
}
*/

// fly pid by my favorite amogh gupta

double fly_kp = 0.25; //increase speed
double fly_ki = 0.2; //range of fluctuation
double fly_kd = 0.00005; //fluctuations
double speed_margin_pct = 2;
bool flyescvar = false;
double speed_margin = 0;
double speed_rpm = 0;

void speed(double targspeedpct) {
  double avgrpm = 0;
  double preverror = 0;
  double error = 0;
  double errorsum = 0;
  double derivative = 0;
  double targspeedrpm = (targspeedpct)*3600;
  wait(10,msec);

  while (!flyescvar) {
    avgrpm = flywheel.velocity(rpm);
    error = targspeedrpm - avgrpm;
    derivative = preverror - error;
    errorsum += error;
    preverror = error;
    speed_margin = fabs((error/targspeedrpm)*100);
    speed_rpm = error * fly_kp + errorsum * fly_ki + derivative * fly_kd;
    wait(5,msec);

    if (speed_margin <= speed_margin_pct) {
      flyescvar = true;
    }
    else {
      flywheel.spin(forward, speed_rpm, rpm);
    }
    wait(10,msec);
  }
}
double speed_volt = 0;
double preverror = 0;
double errorsum = 0;
double error = 0;
double derivative = 0;
void flypid(double flywheel_target_speed_pct) {
  double averagevolt = 0;
  
  double flywheel_target_speed_volt = (flywheel_target_speed_pct/100)*12;

  averagevolt = flywheel.voltage();
  error = flywheel_target_speed_volt - averagevolt;
  derivative = preverror - error;
  errorsum += error;
  preverror = error;
  speed_margin = fabs((error/flywheel_target_speed_volt) * 100);
  speed_volt =  error * fly_kp + fly_ki * errorsum + fly_kd * derivative;
  
  flywheel.spin(fwd, speed_volt, volt);
}

// drive code
void dtcode(double y, double x) {
  double rightspeed = (gamers.Axis3.position() * y) + (gamers.Axis4.position() * -x);
  double leftspeed = (gamers.Axis3.position() * y) - (gamers.Axis4.position() * -x);
  fl.spin(forward, leftspeed, percent);
  ml.spin(forward,leftspeed,percent);
  bl.spin(forward, leftspeed, percent);
  fr.spin(forward, rightspeed, percent);
  mr.spin(forward,rightspeed,percent);
  br.spin(forward, rightspeed, percent);
}



// ..........................................................................
// auton functions
// ..........................................................................
void setV(double x) {
  fl.setVelocity(x, percent);
  ml.setVelocity(x,percent);
  bl.setVelocity(x, percent);
  fr.setVelocity(x, percent);
  mr.setVelocity(x,percent);
  br.setVelocity(x, percent);
}
void setcoast() {
  fl.setStopping(coast);
  ml.setStopping(coast);
  bl.setStopping(coast);
  fr.setStopping(coast);
  mr.setStopping(coast);
  br.setStopping(coast);
}
void Forward(double joe) {
  ml.spin(forward,joe,pct);
  bl.spin(forward,joe,pct);
  mr.spin(forward,joe,pct);
  br.spin(forward,joe,pct);
  fl.spin(forward,joe,pct);
  fr.spin(forward,joe,pct);
}
void Reverse(double joe) {
  ml.spin(reverse,joe,pct);
  bl.spin(reverse,joe,pct);
  mr.spin(reverse,joe,pct);
  br.spin(reverse,joe,pct);
  fl.spin(reverse,joe,pct);
  fr.spin(reverse,joe,pct);
}
void righT(double joe) {
  ml.spin(forward,joe,pct);
  bl.spin(forward,joe,pct);
  mr.spin(reverse,joe,pct);
  br.spin(reverse,joe,pct);
  fl.spin(reverse,joe,pct);
  fr.spin(reverse,joe,pct);
}
void lefT(double joe) {
  ml.spin(reverse,joe,pct);
  bl.spin(reverse,joe,pct);
  mr.spin(forward,joe,pct);
  br.spin(forward,joe,pct);
  fl.spin(forward,joe,pct);
  fr.spin(forward,joe,pct);
}
void For(double x, double y, double z) {
  setV(y);
  fl.spinFor(forward, x, degrees, false);
  ml.spinFor(forward, x, degrees, false);
  bl.spinFor(forward, x, degrees, false);
  fr.spinFor(forward, x, degrees, false);
  mr.spinFor(forward, x, degrees, false);
  br.spinFor(forward, x, degrees);
  wait(z, msec);
}
void Rev(double x, double y, double z) {
  setV(y);
  fl.spinFor(reverse, x, degrees, false);
  ml.spinFor(reverse, x, degrees, false);
  bl.spinFor(reverse, x, degrees, false);
  fr.spinFor(reverse, x, degrees, false);
  mr.spinFor(reverse, x, degrees, false);
  br.spinFor(reverse, x, degrees);
  wait(z, msec);
}
void Revang(double x, double y, double z) {
  fr.spin(reverse, x, percent);
  mr.spin(reverse, x, percent);
  bl.spin(reverse, x, percent);
  fr.spin(reverse, y, percent);
  mr.spin(reverse, y, percent);
  br.spin(reverse, y, percent);
  wait(z, msec);
  fl.stop();
  bl.stop();
  fr.stop();
  br.stop();
}
void Right(double x, double y, double z) {
  setV(y);
  fl.spinFor(forward, x, degrees, false);
  ml.spinFor(forward, x, degrees, false);
  bl.spinFor(forward, x, degrees, false);
  fr.spinFor(reverse, x, degrees, false);
  mr.spinFor(reverse, x, degrees, false);
  br.spinFor(reverse, x, degrees);
  wait(z, msec);
}
void Left(double x, double y, double z) {
  setV(y);
  fl.spinFor(reverse, x, degrees, false);
  ml.spinFor(reverse, x, degrees, false);
  bl.spinFor(reverse, x, degrees, false);
  fr.spinFor(forward, x, degrees, false);
  mr.spinFor(forward, x, degrees, false);
  br.spinFor(forward, x, degrees);
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
  wait(100, msec);
  shooter.set(true);
  wait(y, msec);
}

// low goal and roller right
void lgrRight(bool x) {
  setcoast();
  Inertial.setHeading(0,degrees);
  if(x){
    speed(20);
    wait(2500, msec);
    shoot(500, lowgoal, lowgoal);
    shoot(500, lowgoal, lowgoal);
    x = false;
  }
  flywheel.stop(coast);
  For(500/4,20,20);
  spinny.spin(reverse, 65, percent);
  Right(180,20,20);
  For(180, 20, 120);
  spinny.stop();
  Rev(20,0,0);
  Inertial.resetHeading();
}
// low goal and roller left
void lgrLeft(bool x) {
  setcoast();
  if(x){
    speed(20);
    wait(2500, msec);
    shoot(500, lowgoal, lowgoal);
    shoot(500, lowgoal, lowgoal);
    flywheel.stop(coast);
    x = false;
  }
    spinny.spin(reverse,50,percent);
    For(75,20,200);
    
    //Left(30, 20, 0);
    //For(300, 20, 90);
  }


// low goal and roller and high goal right
void lgrhgRight() {
  Inertial.setHeading(0,degrees);
  /*spinny.spin(forward,100,percent);
  For(500, 30, 1750);
  spinny.stop();
  flypid(199.75);
  Right((225*2-7.6),30,3000);
  shooter.set(false);
  wait(300, msec);
  shooter.set(true);
  wait(300,msec);
  flypid(194.959);
  wait(1000, msec);
  shooter.set(false);
  wait(300, msec);
  shooter.set(true);
  wait(1000, msec);
  shooter.set(false);
  wait(200, msec);
  shooter.set(true);
  wait(500, msec);
  flywheel.stop(coast);*/

  lgrRight(false);
  Rev(154,20,0);
  while(Inertial.heading(degrees) > 225){
    fl.spin(forward, 20, percent);
    bl.spin(forward, 20, percent);
    ml.spin(forward,20,percent);
    fr.spin(reverse, 20, percent);
    br.spin(reverse, 20, percent);
    mr.spin(reverse,20,percent);
  }
  flypid(188);
  spinny.spin(forward, 100, percent);
  For(1000,50,1000);
  Left(190, 20, 0);
  Rev(60,20,200);
  shooter.set(false);
  wait(200, msec);
  shooter.set(true);
  wait(1000, msec);
  shooter.set(false);
  wait(200, msec);
  shooter.set(true);
  wait(1000, msec);
  shooter.set(false);
  wait(200, msec);
  shooter.set(true);
  wait(500, msec);
  flywheel.stop(coast);
  spinny.stop();
}
// low goal and roller and high goal left
void lgrhgLeft() {
  setcoast();
  lgrLeft(false);
  Rev(70,20,0);
  Left(300, 50, 0);
  spinny.spin(forward, 100, percent);
  For(650,100,1000);
  Right(223, 20, 100);
  Rev(40,20,20);
  spinny.spin(reverse,100,percent);
  flypid(165);
  wait(2000,msec);
  spinny.stop();
  shooter.set(false);
  wait(200, msec);
  shooter.set(true);
  wait(500, msec);
  shooter.set(false);
  wait(200, msec);
  shooter.set(true);
  wait(500, msec);
  shooter.set(false);
  wait(200, msec);
  shooter.set(true);
  wait(500, msec);
  flywheel.stop(coast);
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
  Inertial.calibrate();
  while(Inertial.isCalibrating()) {
    wait(20,msec);
  }
  setcoast();
  shooter.set(true);
  fl.setPosition(0,degrees);
  fr.setPosition(0,degrees);
  mr.setPosition(0,degrees);
  ml.setPosition(0,degrees);
  bl.setPosition(0,degrees);
  br.setPosition(0,degrees);
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
    int df = flywheel.velocity(percent);
    double goofygoober = flywheel.temperature(celsius);
    gamers.Screen.clearScreen();
    gamers.Screen.setCursor(1, 1);
    gamers.Screen.print(df);
    gamers.Screen.setCursor(3,1);
    gamers.Screen.print(goofygoober);
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
      dtcode(1, 0.35);
    }

    // ..........................................................................
    // intake/roller
    // ..........................................................................
    if (gamers.ButtonUp.pressing()) {
      spinny.spin(forward, 100, percent);
    } 
    else if (gamers.ButtonDown.pressing()) {
      spinny.spin(reverse, 100, percent);
    } 
    else if (gamers.ButtonLeft.pressing()) {
      spinny.stop(coast);
    }

    // ..........................................................................
    // shooter/indexer
    // ..........................................................................
    // shooter

    if (toggle) {
      flypid(75);
    } 
    else {
      flywheel.stop(coast);
    }

    if (gamers.ButtonA.pressing()) {
      if (!latch) {
        toggle = !toggle;
        latch = true;
      }
    } 
    else {
      latch = false;
    }
    // shooter
    if (gamers.ButtonL1.pressing()) {
      shooter.set(false);
      wait(200,msec);
      shooter.set(true);
    }
    if(toggle) {
      gamers.rumble(rumblePulse);
    }
    // ..........................................................................
    //
    // ..........................................................................
    if (gamers.ButtonL1.pressing() && gamers.ButtonL2.pressing() &&
        gamers.ButtonR2.pressing() && gamers.ButtonR1.pressing()) {
      expansion.set(true);
      wait(5,msec);
      gamers.Screen.clearScreen();
      gamers.Screen.setCursor(1, 1);
      gamers.Screen.print("expansion fired");
      flywheel.stop(coast);
    }
   /* else if (gamers.ButtonUp.pressing() && gamers.ButtonDown.pressing() &&
        gamers.ButtonRight.pressing() && gamers.ButtonLeft.pressing()) {
      expansion.spin(reverse, 50, percent);
      spinny.stop(coast);
    }
     else {
      expansion.stop();
    }*/
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