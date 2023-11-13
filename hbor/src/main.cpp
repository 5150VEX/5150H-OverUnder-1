#include "main.h"
#include "lemlib/api.hpp"
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
    100, // kP
    0, // kD
    1, // smallErrorRange
    10000, // smallErrorTimeout
    3, // largeErrorRange
    50000, // largeErrorTimeout
    5 // slew rate
};

// turning PID
// left and right pid for smooth turning (the one piece of code that we have been struggling with for a while. 
// I figured out that tunning the PID on a hardwood floor was the issue and we are now using the tiles to tune it)
lemlib::ChassisController_t angularController {
    0.75, // kP 0.75
    60, // kD 72.8
    1, // smallErrorRange
    100, // smallErrorTimeout
    25, // largeErrorRange
    500, // largeErrorTimeout
    5
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
	chassis.turnTo(sin(deg * (PI/180))*1000, cos(deg * (PI/180))*1000, timeout, false, speed);
}

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading

        pros::delay(10);
    }
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "5150 HAVOC");

	chassis.calibrate();
	chassis.setPose(0, 0, 0);

	pros::Task screenTask(screen);
}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {
    chassis.turnTo(10000, 0, 5000, false);   // pid
	// turnTo(90, 127, 5000);                   // my function

    // drive    forward     1200
    // drive    reverse     800

    // turn     right       360

    // drive    reverse     900
    // drive    forward     400

    // wait     seconds     0.5

    // drive    reverse     500
    // drive    forward     800

    // turn     right       360

    // drive    reverse     1000
    // turn     left        100
    // drive    reverse     1300
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

        // lag is real. this prevents it.
        pros::delay(5);
    }
}
