#include "main.h"
#include "lemlib/api.hpp"
#include "autoSelect/selection.h"
#include "hbot.hpp"
#include <iostream>
#include <string>

//==================================================// Red   : 36
// PHYSICAL OBJECTS INITS 		               		// Green : 12
//==================================================// Blue  : 06

pros::Controller controller(pros::E_CONTROLLER_MASTER); // controller setup

pros::ADIDigitalOut wing ('g', false); // wing setup
pros::ADIDigitalOut endgame ('h', false); // endgame setup

pros::Motor cata(20, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES); // cata motor
pros::Motor intakeMotor(2, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES); // intake motor

// drivebase setup
// This initiallizes the four motors we use for the drive base
pros::Motor lFmotor(14, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lBmotor(17, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rFmotor(7, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rBmotor(6, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

// creates two motor groups that allow me to use "left" and "right" to control... 
// the left and right side of the robot respectively
pros::MotorGroup left({lFmotor, lBmotor});
pros::MotorGroup right({rFmotor, rBmotor});

// setting up the drivebase so i can call move commands in order to move and turn the entire bot
lemlib::Drivetrain_t drivetrain {
    &left, // left drivetrain motors
    &right, // right drivetrain motors
    12, // track width
    3.25, // wheel diameter
    360 // wheel rpm
};

// inertial sensor
// this sets up the inertial sensor so we can use PID properly
pros::Imu gyro(16);

// odometry struct
// sets up the "odometry wheels" and the inertial sensor...
// though we dont use odometry so we declare the wheels as null
lemlib::OdomSensors_t sensors {
	nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &gyro // inertial sensor
};

// forward/backward PID
// forward backward pid for smooth lateral movment
lemlib::ChassisController_t lateralController {
    8, // kP
    64, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    2, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};

// turning PID
// left and right pid for smooth turning
lemlib::ChassisController_t angularController {
    .kP = -8, // kP 0.75
    .kD = -64, // kD 64
    .smallError = 0.5, // smallErrorRange
    .smallErrorTimeout = 100, // smallErrorTimeout
    .largeError = 1, // largeErrorRange
    .largeErrorTimeout = 500, // largeErrorTimeout
    .slew = 5
};
	
// create the chassis
// declare the robot to use in auton for PID
lemlib::Chassis chassis(drivetrain, lateralController, angularController,sensors);

//==================================================//
// VARS				            	 				//
//==================================================//

bool wingFlag = true;
bool wingState = false;
bool endFlag = true;
bool endState = false;
const double PI = 3.14159;

//==================================================//
// VOIDS				 	            			//
//==================================================//

//==================================================//
// v v v CATA CONTROLS                        v v v //
void cataControl(){
    // define the right two bumpers and triggers on the controller as booleans
	bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
	bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

	if (r2){
		cata.move(127);
	} else if (r1) {
		cata.move(64);
    } else {
		cata.brake();
	}
};
// ^ ^ ^ CATA CONTROLS                        ^ ^ ^ //
//==================================================//

//==================================================//
// v v v INTAKE CONTROLS                      v v v //
void intakeControl(){
    // declares the up and down buttons on the controller as booleans (one for intake and one for outtake)
	bool bUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
	bool bDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    
    // define a speed for the intake roller
    int intakeSpeed = 64;

    // if the up arrow is pressed, intake. if the down button is pressed, outtake. else, stop the motor from turning.
	if (bUp){ // intake
		intakeMotor.move(-intakeSpeed);
	} else if (bDown) { // outtake
		intakeMotor.move(intakeSpeed);
    } else {
		intakeMotor.brake();
	}
};
void intake(int speed){
    intakeMotor.move_relative(90, speed);
};
void outtake(int speed){
    intakeMotor.move_relative(-90, speed);
};
// ^ ^ ^ INTAKE CONTROLS                      ^ ^ ^ //
//==================================================//

//==================================================//
// v v v WING CONTROLS                        v v v //
void wingControl(){
	bool ba = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    
    // setting up a toggle for the wing
	if (wingFlag && ba) {
		wingFlag = false;
        if (wingState){
            wingState = false;
		    wing.set_value(false);
        } else {
            wingState = true;
            wing.set_value(true);
        }
	}

    // the magic of the toggle function
    // the wingFlag, once set to false, will only set to true again if the A button...
    // ... isn't pressed, this prevents repeated calls to open and close the wing
	if (!ba) {
		wingFlag = true;
	}
};
void wingSet(bool state){
    wing.set_value(state);
    wingState = state;
};
// ^ ^ ^ WING CONTROLS                        ^ ^ ^ //
//==================================================//

//==================================================//
// v v v ENDGAME CONTROLS                     v v v //
void endgameControl(){
	bool l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    // setting up a toggle for the hanger
	if (endFlag && l1) {
		endFlag = false;

        if (endState){
            endState = false;
		    endgame.set_value(false);
        } else {
            endState = true;
            endgame.set_value(true);
        }
	}

    // the endFlag, once set to false, will only set to true again if the left bumper...
    // ... isn't pressed, this prevents repeated calls to open and close the wing
	if (!l1) {
		endFlag = true;
	}
};

void endgameSet(bool state){
    endgame.set_value(state);
    endState = state;
};
// ^ ^ ^ ENDGAME CONTROLS                     ^ ^ ^ //
//==================================================//

void turnTo(float deg, float speed, int timeout){
    // using trig to turn a degree into a point that is far away from the bot in...
    // ... order to remove the need for odometry
	chassis.turnTo(sin(deg * (PI/180))*1000, cos(deg * (PI/180))*1000, timeout, false, speed);
}
void turnTo(float deg){
    // using trig to turn a degree into a point that is far away from the bot in...
    // ... order to remove the need for odometry
	chassis.turnTo(sin(deg * (PI/180))*1000, cos(deg * (PI/180))*1000, 5000, false, 127);
}

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading

        pros::delay(10); // saves memory
    }
}

void initialize() {
	pros::lcd::initialize(); 
	pros::lcd::set_text(1, "5150 HAVOC");

    selector::init();

	chassis.calibrate(); // using this for auton
	chassis.setPose(0, 0, 0); // sets the origin to where we place the bot

	pros::Task screenTask(screen);

}

/* diasbled & competition_initialize
void disabled() {

}

void competition_initialize() {

}
*/

void autonomous() {
    // turnTo(90, 127, 5000);
    // points generated with https://5150vex.github.io/path.jerryio/

    /*
    // LEFT SIDE AUTON
    chassis.moveTo(0, 0, 5000);
    chassis.moveTo(18.116, 15.985, 5000);
    chassis.moveTo(28.773, 15.985, 2000);
    chassis.moveTo(0, -8.312, 5000);
    chassis.moveTo(2, -33, 5000);

    // RIGHT SIDE AUTON
    chassis.moveTo(0, 0, 5000);
    chassis.moveTo(-29.257, 20.144, 2000);
    chassis.moveTo(-0.24, -6.955, 5000);
    chassis.moveTo(-0.719, -33.814, 5000);
    */
    if(selector::auton == 1){
        turnTo(90);
    }
    if(selector::auton == 2){
        turnTo(-90);
    }
}

void opcontrol() {

	while (true) {
        auto turn = controller.get_analog(ANALOG_RIGHT_X);
        auto power = controller.get_analog(ANALOG_LEFT_Y);

        // this turns the two stick into rotational and lateral movement
        left.move(power - turn);
		right.move(power + turn);

        // allows the voids to do their things (without this, the macros would not work)
		cataControl();
		wingControl();
		endgameControl();
		intakeControl();

        //saves a bit of memory
        // there is less waiting because this void manages drive code and we cant have...
        // ... input delay
        pros::delay(5);
    }
}

/*

  ／|、
（ﾟ､ ｡７
 |、ﾞ~ヽ
 じしf._)ノ 
 
 her name is soy sauce
 
*/