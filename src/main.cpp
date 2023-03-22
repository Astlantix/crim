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

void speeed(double target) {
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

// my pid
// Define the PID constants
double fly_kp = 0.2;
double fly_ki = 1;
double fly_kd = 0.4;

// Define the error threshold
double error_threshold = 10;

// Define the timeout
double timeout = 5; // in seconds

// This is the function that we will be calling in our main program
void speed(double targspeedpct) {
  // Convert target speed percentage to RPM
  double targspeedrpm = targspeedpct * 3600 / 100;

  // Set up variables for PID control and EMA filter
  double avgrpm = 0;
  double filtered_velocity = 0;
  double preverror = 0;
  double error = 0;
  double errorsum = 0;
  double derivative = 0;
  double alpha = 0.2; // set the filter coefficient

  // Set a timer for the timeout
  vex::timer t;
  t.reset();
  t.time();

  // Run the PID control loop until the error is below the threshold or the timeout has elapsed
  while (fabs(error) > error_threshold && t.time() < timeout) {
    // Calculate the average RPM of the flywheel
    avgrpm = flywheel.velocity(rpm);

    // Apply the EMA filter to smooth out the velocity readings
    filtered_velocity = alpha * avgrpm + (1 - alpha) * filtered_velocity;

    // Calculate the error between the filtered velocity and the target velocity
    error = targspeedrpm - filtered_velocity;

    // Calculate the derivative of the error
    derivative = error - preverror;

    // Add the error to the error sum
    errorsum += error;

    // Set the previous error to the current error
    preverror = error;

    // Calculate the flywheel speed based on the PID controller
    double speed_rpm = error * fly_kp + errorsum * fly_ki + derivative * fly_kd;

    // Set the flywheel speed
    flywheel.spin(forward, speed_rpm, rpm);

    // Wait for a short time to allow the flywheel to stabilize
    wait(5, msec);
  }

  // Stop the flywheel when the target speed has been reached
  flywheel.stop();
}


// fly pid by my favorite amogh gupta
double avgrpm = 0;
double preverror = 0;
double error = 0;
double errorsum = 0;
double derivative = 0;
double speed_margin = 2;
double speed_volt = 0; //create a variable to hold the speed in voltage
/*double preverror = 0; //create a variable to hold the previous error
double errorsum = 0; //create a variable to hold the sum of errors
double error = 0; //create a variable to hold the current error
double derivative = 0; //create a variable to hold the derivative*/
void flypid(double flywheel_target_speed_pct) { //create a function to take the target speed in percent
  double averagevolt = 0; //create a variable to hold the average voltage
  
  double flywheel_target_speed_volt = (flywheel_target_speed_pct/100)*12; //calculate the target speed in voltage

  averagevolt = flywheel.voltage(); //measure the average voltage
  error = flywheel_target_speed_volt - averagevolt; //calculate the error
  derivative = preverror - error; //calculate the derivative
  errorsum += error; //add the error to the sum of errors
  preverror = error; //set the previous error to the current error
  speed_margin = fabs((error/flywheel_target_speed_volt) * 100); //calculate the margin of error as a percentage
  speed_volt =  error * fly_kp + fly_ki * errorsum + fly_kd * derivative; //calculate the speed in voltage
  
  flywheel.spin(fwd, speed_volt, volt); //spin the flywheel at the calculated speed
}

// drive code
void dtcode(double y, double x) { //define the function
  double rightspeed = (gamers.Axis3.position() * y) + (gamers.Axis4.position() * -x); //calculate the right speed
  double leftspeed = (gamers.Axis3.position() * y) - (gamers.Axis4.position() * -x); //calculate the left speed
  fl.spin(forward, leftspeed, percent); //set the left motor speed
  ml.spin(forward,leftspeed,percent); //set the left motor speed
  bl.spin(forward, leftspeed, percent); //set the left motor speed
  fr.spin(forward, rightspeed, percent); //set the right motor speed
  mr.spin(forward,rightspeed,percent); //set the right motor speed
  br.spin(forward, rightspeed, percent); //set the right motor speed
}
void stoop() {
  ml.stop(hold);
  mr.stop(hold);
  fr.stop(hold);
  fl.stop(hold);
  bl.stop(hold);
  br.stop(hold);
}

void RIGHT(double amount) {
  amount -= 6;
Inertial.setRotation(0, degrees);
while(fabs(Inertial.rotation(degrees)) < amount) {
  double error = amount - fabs(Inertial.rotation(degrees));
  fl.spin(forward, 0.2*error + 4, percent);
  ml.spin(forward, 0.2*error + 4, percent);
  bl.spin(forward, 0.2*error + 4, percent);
  fr.spin(reverse, 0.2*error + 4, percent);
  mr.spin(reverse, 0.2*error + 4, percent);
  br.spin(reverse, 0.2*error + 4, percent);
  wait(5, msec);
}
stoop();
wait(20, msec);
Inertial.setRotation(0, degrees);
}

void LEFT(double amount) {
  amount -= 6;
Inertial.setRotation(0, degrees);
while(fabs(Inertial.rotation(degrees)) < amount) {
  double error = amount - fabs(Inertial.rotation(degrees));
  
  fl.spin(reverse, 0.2*error + 5, percent);
  ml.spin(reverse, 0.2*error + 5, percent);
  bl.spin(reverse, 0.2*error + 5, percent);
  fr.spin(forward, 0.2*error + 5, percent);
  mr.spin(forward, 0.2*error + 5, percent);
  br.spin(forward, 0.2*error + 5, percent);
  wait(5, msec);
}
stoop();
wait(20, msec);
Inertial.setRotation(0, degrees);
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
  wait(0, msec);
}
void Rev(double x, double y, double z) {
  setV(y);
  fl.spinFor(reverse, x, degrees, false);
  ml.spinFor(reverse, x, degrees, false);
  bl.spinFor(reverse, x, degrees, false);
  fr.spinFor(reverse, x, degrees, false);
  mr.spinFor(reverse, x, degrees, false);
  br.spinFor(reverse, x, degrees);
  wait(0, msec);
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
  if(x){
    flypid(65);
    wait(2500, msec);
    shoot(500, lowgoal, lowgoal);
    shoot(500, lowgoal, lowgoal);
    x = false;
  }
  flywheel.stop(coast);
  For(500,20,20);
  spinny.spin(reverse, 65, percent);
  RIGHT(93);
  For(178, 20, 1400);
  spinny.stop();
  Rev(178,20,0);
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
  spinny.spin(forward,100,percent);
  For(500, 45, 1750);
  flypid(101.2);
  LEFT(163);
  wait(600,msec);
  shooter.set(false);
  wait(100, msec);
  shooter.set(true);
  wait(3000, msec);
  shooter.set(false);
  wait(100, msec);
  shooter.set(true);
  wait(2500, msec);
  shooter.set(false);
  wait(100, msec);
  shooter.set(true);
  wait(150, msec);
  flywheel.stop(coast);
  LEFT(76);
  spinny.spin(reverse,100,pct);
  For(590, 70, 1);
  RIGHT(60);
  For(260,30,20);
  wait(0.5, sec);
  spinny.stop();
  /*lgrRight(false);
  Rev(154,20,0);
  RIGHT(135);
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
  spinny.stop();*/
  setcoast();
}
// low goal and roller and high goal left
void lgrhgLeft() {
  /*setcoast();
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
  wait(175, msec);
  shooter.set(true);
  wait(50, msec);
  shooter.set(false);
  wait(175, msec);
  shooter.set(true);
  wait(50, msec);
  shooter.set(false);
  wait(175, msec);
  shooter.set(true);
  wait(500, msec);
  flywheel.stop(coast);*/
  flypid(102.98);
  wait(4,sec);
  shooter.set(false);
  wait(100,msec);
  shooter.set(true);
  wait(3000,msec);
  shooter.set(false);
  wait(100,msec);
  shooter.set(true);
  wait(300,msec);
  flywheel.stop(coast);
  spinny.spin(reverse,65,percent);
  For(100,20,750);
  wait(5,sec);
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
  a = false;
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
  setcoast();
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
double dspeed = 75;
bool goofy;
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
  while (!a) {
    // ..........................................................................
    // printing temp/speed
    // ..........................................................................
    int df = flywheel.velocity(rpm);
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


    if(gamers.ButtonA.pressing()) {
      dspeed = 75;
      goofy = false;
    }

    else if (gamers.ButtonX.pressing()) {
      dspeed = 65;
      goofy = false;
    }
    else if (gamers.ButtonY.pressing()) {
      dspeed = 90;
      goofy = true;
    }

    if (toggle) {
      if(goofy) {
        flypid(dspeed);
      }
      else {
        flypid(dspeed);
      }
    } 
    else {
      flywheel.stop(coast);
    }

    if (gamers.ButtonA.pressing() || gamers.ButtonY.pressing() || gamers.ButtonX.pressing()) {
      if (!latch) {
        toggle = 1;
        latch = true;
      }
    } 
    else if(gamers.ButtonB.pressing()) {
      latch = false;
      toggle = false;
    }
    // shooter
    if (gamers.ButtonL1.pressing()) {
      shooter.set(false);
      wait(175,msec);
      shooter.set(true);
      wait(50,msec);
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