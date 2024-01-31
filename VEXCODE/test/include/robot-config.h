using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LF;
extern motor LB;
extern motor RF;
extern motor RB;
extern controller Controller1;
extern motor Motor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );