#include "vex.h"
#include "math.h"
#include <iostream>

/*gary the snail*/



// adjusts value of x-coordinate
double xposchange () {
    double currentpos = xp.position(degrees);
    xp.setPosition(0,degrees);
    return currentpos*23;
}
// returns value of y-coordinate
double yposchange() {
    double currentpos = yp.position(degrees);
    yp.setPosition(0,degrees);
    return currentpos*23;
}

double xpos;
double ypos;
double heading;
void odometry(double startx, double starty) {
    xpos = startx;
    ypos = starty;
    heading = Inertial.rotation(degrees);
    while(1) {
        xpos += xposchange();
        ypos += yposchange();
        heading = Inertial.rotation(degrees);
    }

    
    // update the positions
}