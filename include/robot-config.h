using namespace vex;


extern brain Brain;

// VEXcode devices
extern motor fr;
extern motor mr;
extern motor br;
extern motor fl;
extern motor ml;
extern motor bl;
extern motor spinny;
extern motor_group flywheel;
extern inertial Inertial;
extern optical Optical;
extern digital_out shooter;
extern controller gamers;
extern motor expansion;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );