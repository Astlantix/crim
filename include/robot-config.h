using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor rf;
extern motor rb;
extern motor lf;
extern motor lb;
extern motor roller;
extern motor_group fly_wheel;
extern inertial Inertial;
extern optical Optical;
extern digital_out indexer;
extern controller Controller1;
extern motor expansion;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );