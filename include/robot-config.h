using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern drivetrain Drivetrain;
extern motor top;
extern motor Intake;
extern motor storage;
extern digital_out scraper;
extern bool alliance;
extern char colora;
extern int speed;
extern int fishSpeed;
extern int fishPosition;
extern bool goalExtended;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );