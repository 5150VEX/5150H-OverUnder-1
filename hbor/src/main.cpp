#include "main.h"
#include "lemlib/api.hpp"
#include "hbot.hpp"

//======================================// Red   : 36
// PHYSICAL OBJECTS INITS 				// Green : 12
//======================================// Blue  : 06

pros::Controller controller(pros::E_CONTROLLER_MASTER); // controller setup

pros::Motor cata(20, pros::E_MOTOR_GEARSET_36, false); // cata motor

// drivebase setup
pros::Motor lFmotor(14, pros::E_MOTOR_GEARSET_06, false);
pros::Motor lBmotor(17, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rFmotor(7, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rBmotor(6, pros::E_MOTOR_GEARSET_06, true);

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
pros::Imu inertial_sensor(2); // port 2

// odometry struct
lemlib::OdomSensors_t sensors {
	// fuck you bozo
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    40 // slew rate
};
	
	
// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController,sensors);

//======================================//
// VARS					 				//
//======================================//

bool wingOn = false;

/*
A callback function for LLEMU's center button.
 
When this callback is fired, it will toggle line 2 of the LCD text between
"I was pressed!" and nothing.
*/
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void cataLaunch(){
	if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
		cata.move(-100);
		//cout("cata is running");
	} else {
		cata.brake();
	}
};

/*
Runs initialization code. This occurs as soon as the program is started.

All other competition modes are blocked by initialize; it is recommended
to keep execution time for this mode under a few seconds.
*/
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	//chassis.moveTo(float x, float y, int timeout);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
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

		
        pros::delay(5);
    }
}
