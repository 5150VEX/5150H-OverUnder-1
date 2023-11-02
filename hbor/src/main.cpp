#include "main.h"
#include "lemlib/api.hpp"
#include "hbot.hpp"
#include <iostream>
#include <string>

//======================================// Red   : 36
// PHYSICAL OBJECTS INITS 				// Green : 12
//======================================// Blue  : 06

pros::Controller controller(pros::E_CONTROLLER_MASTER); // controller setup

pros::Motor cata(20, pros::E_MOTOR_GEARSET_36, false); // cata motor
pros::Motor intakeMotor(2, pros::E_MOTOR_GEARSET_36, false); // intake motor

// drivebase setup
pros::Motor lFmotor(14, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lBmotor(17, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rFmotor(7, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rBmotor(6, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::MotorGroup left({lFmotor, lBmotor});
pros::MotorGroup right({rFmotor, rBmotor});

lemlib::Drivetrain_t drivetrain {
    &left, // left drivetrain motors
    &right, // right drivetrain motors
    12, // track width
    3.25, // wheel diameter
    360 // wheel rpm
};

/*
// left tracking wheel encoder
pros::ADIEncoder left_enc('A', 'B', true); // ports A and B, reversed
// right tracking wheel encoder
pros::Rotation right_rot(1, false); // port 1, not reversed
// back tracking wheel encoder
pros::ADIEncoder back_enc('C', 'D', false); // ports C and D, not reversed

// left tracking wheel
lemlib::TrackingWheel left_tracking_wheel(&left_enc, 2.75, -4.6); // 2.75" wheel diameter, -4.6" offset from tracking center
// right tracking wheel
lemlib::TrackingWheel right_tracking_wheel(&right_rot, 2.75, 1.7); // 2.75" wheel diameter, 1.7" offset from tracking center
lemlib::TrackingWheel back_tracking_wheel(&back_enc, 2.75, 4.5); // 2.75" wheel diameter, 4.5" offset from tracking center
*/
	
// inertial sensor
pros::Imu gyro(16);

// odometry struct
lemlib::OdomSensors_t sensors {
	nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &gyro // inertial sensor
};

// forward/backward PID
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
lemlib::ChassisController_t angularController {
    1.6, // kP
    128, // kD 110 128
    3, // smallErrorRange
    100, // smallErrorTimeout
    5, // largeErrorRange
    500, // largeErrorTimeout
    5
};
	
	
// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController,sensors);

//======================================//
// VARS					 				//
//======================================//

bool wingFlag = true;
bool wingOn = false;
float moveBais = 1;
const double PI = 3.14159;

//======================================//
// VOIDS				 				//
//======================================//

void cataLaunch(){
	bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
	bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

	if (r2 && r1){
		cata.move(-64);
	} else if (r2){
			cata.move(-127);
	} else {
		cata.brake();
	}
};

void intake(){
	bool bUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
	bool bDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    
    int intakeSpeed = 64;

	if (bUp){ // intake
		intakeMotor.move(-intakeSpeed);
	} else if (bDown) { // outake
		intakeMotor.move(intakeSpeed);
    } else {
		intakeMotor.brake();
	}
};

void wingControl(){
	bool ba = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
	pros::ADIDigitalOut wing ('g', false);

	if (wingFlag && ba) {
		wingFlag = false;
		wing.set_value(!wingOn);
	}

	if (!ba) { 
		wingFlag = true;
	}
};

void turnTo(float deg, float speed, int timeout){
	chassis.turnTo(cos(deg * (PI/180)), sin(deg * (PI/180)), timeout, false, speed);
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


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

void autonomous() {
	chassis.turnTo(30, 0, 5000, false);

	//turnTo(180, 127, 500);
}

void opcontrol() {

	while (true) {
        auto turn = controller.get_analog(ANALOG_RIGHT_X);
        auto power = controller.get_analog(ANALOG_LEFT_Y);
        left.move(power - turn);
		right.move(power + turn);
		/*
        if (controller->pressed(DIGITAL_R1) && 
			controller->pressed(DIGITAL_R2) && 
			controller->pressed(DIGITAL_L1) && 
			controller->pressed(DIGITAL_L2)) {
            robot->endgame->fire();
        }
		*/

		cataLaunch();
		wingControl();
		intake();

        pros::delay(5);
    }
}
