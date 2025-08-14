#include "vex.h"
#include "robot-config.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT18, ratio18_1, false);
motor leftMotorB = motor(PORT19, ratio18_1, true);
motor leftMotorC = motor(PORT20, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT1, ratio18_1, true);
motor rightMotorB = motor(PORT2, ratio18_1, false);
motor rightMotorC = motor(PORT3, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
motor storage = motor(PORT13, false);
motor Intake = motor(PORT12, ratio18_1, false);
motor top = motor(PORT11, false);
digital_out scraper = digital_out(Brain.ThreeWirePort.G);
digital_out aligner = digital_out(Brain.ThreeWirePort.F);
line Color = line(Brain.ThreeWirePort.D);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
bool goalExtended = true;
bool alignerExtended = false;
// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 7 && drivetrainLeftSideSpeed > -7) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 7 && drivetrainRightSideSpeed > -7) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonR1/R2/L1/L2 status to control Conveyer
      if (Controller1.ButtonL1.pressing()) {
        storage.spin(fwd, 80, percentUnits::pct);
        top.spin(reverse, 80, percentUnits::pct);
        Intake.spin(reverse, 80, percentUnits::pct);
      } else if (Controller1.ButtonL2.pressing()) {
        storage.spin(fwd, 80, percentUnits::pct);
        Intake.spin(reverse, 80, percentUnits::pct);
        top.spin(fwd, 80, percentUnits::pct);
      } else if (Controller1.ButtonR1.pressing()) {
        storage.spin(reverse, 80, percentUnits::pct);
        top.spin(reverse, 80, percentUnits::pct);
        Intake.spin(reverse, 80, percentUnits::pct);
      } else if (Controller1.ButtonR2.pressing()) {
        storage.spin(fwd, 80, percentUnits::pct);
        Intake.spin(fwd, 80, percentUnits::pct);
        top.spin(fwd, 80, percentUnits::pct);
      } else {
        storage.stop(hold);
        Intake.stop(hold);
        top.stop(hold);
      }
      
      if (Controller1.ButtonDown.PRESSED) {
          scraper.set(false);
          goalExtended = false;
        } else if (Controller1.ButtonUp.PRESSED) {
            // If the button is pressed, extend the scraper
            scraper.set(true);
            goalExtended = true;
      }
      if (Controller1.ButtonA.PRESSED){
        aligner.set(true);
        alignerExtended = true;
      } else if (Controller1.ButtonB.PRESSED) {
        aligner.set(false);
        alignerExtended = false;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}