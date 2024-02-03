#include "main.h"
#include "lemlib/api.hpp"
#include "autoSelect/selection.h"
#include "hbot.hpp"
#include <iostream>
#include <string>

//==================================================// Red   : 36
// PHYSICAL OBJECTS INITS 		               		// Green : 18
//==================================================// Blue  : 06

pros::Controller controller(pros::E_CONTROLLER_MASTER); // controller setup

// inertial sensor
// this sets up the inertial sensor so we can use PID properly
pros::Imu gyro(15);

pros::ADIDigitalOut wing ('a', false); // wing setup
pros::ADIDigitalOut sideHang ('c', false); // sideHang setup

// extra motors
pros::Motor intakeMotor(10, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES); // intake motor
pros::Motor puncherMotor(20, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES); // intake motor

// drivebase setup
// This initiallizes the four motors we use for the drive base
pros::Motor rFmotor( 8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rMmotor( 2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rBmotor(12, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor lFmotor(9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lMmotor(6, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lBmotor(7, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

// creates two motor groups that allow me to use "left" and "right" to control... 
// the left and right side of the robot respectively
pros::MotorGroup left ({lFmotor, lMmotor, lBmotor});
pros::MotorGroup right({rFmotor, rMmotor, rBmotor});

// setting up the drivebase so i can call move commands in order to move and turn the entire bot
lemlib::Drivetrain drivetrain {
    &left, 
    &right,
    11.25,
    3.25,
    360,
    360
};

// odometry struct
// sets up the "odometry wheels" and the inertial sensor...
// though we dont use odometry so we declare the wheels as null
lemlib::OdomSensors sensors {
	nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &gyro // inertial sensor
};

// forward/backward PID
// forward backward pid for smooth lateral movment
lemlib::ControllerSettings lateralController {
    16,
    0,
    64,
    0,
    1,
    1000,
    2,
    5000,
    5
};

// turning PID
// left and right pid for smooth turning
lemlib::ControllerSettings angularController {
    -6,
    0,
    -40,
    0,
    0,
    1000,
    0,
    5000,
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

bool puncherFlag = true;
bool puncherState = false;

bool blockerFlag = true;
bool blockerState = false;

bool sideHangFlag = true;
bool sideHangState = false;

const double PI = 3.14159;

//==================================================//
// VOIDS				 	            			//
//==================================================//

void getSilly(){ // PLEASE don't use this, it literally just crashes the bot...
    int* important = (int*)malloc;
        *important = 0;
};

//==================================================//
// v v v AUTO STUFF                           v v v //
void turnTo(float deg, float speed, int timeout){
    // using trig to turn a degree into a point that is far away from the bot in...
    // ... order to remove the need for odometry
	chassis.turnTo(sin(deg * (PI/180))*1000, cos(deg * (PI/180))*1000, timeout, true, speed, false);
}
void turnTo(float deg){
    // using trig to turn a degree into a point that is far away from the bot in...
    // ... order to remove the need for odometry
	chassis.turnTo(sin(deg * (PI/180))*1000, cos(deg * (PI/180))*1000, 1200);
}



void moveFor(pros::MotorGroup& motorgroup, int milliseconds, int voltage){
    int start = pros::millis();
    while((pros::millis() - start) <= milliseconds){
        motorgroup.move(voltage);
    }
    motorgroup.move(0);
}
void moveFor(pros::Motor motor, int milliseconds, int voltage){
    int start = pros::millis();
    while((pros::millis() - start) <= milliseconds){
        motor.move(voltage);
    }
    motor.move(0);
}

void setOrigin(bool async){
    if(!async){
        chassis.waitUntilDone();
    }
    chassis.setPose(0, 0, 0); // sets the origin to where we place the bot
}
void setOrigin(){
    chassis.setPose(0, 0, 0); // sets the origin to where we place the bot
}
// ^ ^ ^ AUTO STUFF                           ^ ^ ^ //
//==================================================//

//==================================================//
// v v v PUNCHER CONTROLS                     v v v //
void puncherControl(){
    // define the right two bumpers and triggers on the controller as booleans
	bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    
    if (r2) { // if r2 is held
        puncherMotor.move(127);
    }
    else {
        puncherMotor.move(0);
    }
};
// ^ ^ ^ PUNCHER CONTROLS                     ^ ^ ^ //
//==================================================//

//==================================================//
// v v v INTAKE CONTROLS                      v v v //
void intakeControl(){
    // declares the up and down buttons on the controller as booleans (one for intake and one for outtake)
	bool bL1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
	bool bL2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    
    // define a speed for the intake roller
    int intakeSpeed = 64;

    // if the up arrow is pressed, intake. if the down button is pressed, outtake. else, stop the motor from turning.
	if (bL2){ // intake
		intakeMotor.move(-intakeSpeed);
	} else if (bL1) { // outtake
		intakeMotor.move(intakeSpeed);
    } else {
		intakeMotor.brake();
	}
};
void intake(int speed){
    intakeMotor.move(speed);
};
void outtake(int speed){
    intakeMotor.move(-speed);
};
// ^ ^ ^ INTAKE CONTROLS                      ^ ^ ^ //
//==================================================//

//==================================================//
// v v v WING CONTROLS                        v v v //
void wingControl(){
	bool bA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    
    // setting up a toggle for the wing
	if (wingFlag && bA) {
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
	if (!bA) {
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
// v v v SIDE HANG CONTROLS                   v v v //
void sideHangControl(){
	bool bDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

    // setting up a toggle for the hanger
	if (sideHangFlag && bDown) {
		sideHangFlag = false;

        if (sideHangState){
            sideHangState = false;
		    sideHang.set_value(false);
        } else {
            sideHangState = true;
            sideHang.set_value(true);
        }
	}

    // the blockerFlag, once set to false, will only set to true again if the left bumper...
    // ... isn't pressed, this prevents repeated calls to open and close the wing
	if (!bDown) {
		sideHangFlag = true;
	}
};
// ^ ^ ^ SIDE HANG CONTROLS                   ^ ^ ^ //
//==================================================//

//==================================================//
// v v v TURN POWER CONTROLS                  v v v //
float turnPower(){
	bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
	bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

    if (r2) {
        return(1.00);
    } else if (r1) {
        return(0.5);
    } else {
        return(0.75);

    }
};
// ^ ^ ^ TURN POWER CONTROLS                  ^ ^ ^ //
//==================================================//

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading

        controller.print(0, 0, "theta: %f", pose.theta); // print the heading
        controller.print(1, 0, "x: %f", pose.x); // print the x position
        controller.print(2, 0, "y: %f", pose.y); // print the y position
        
        /*
        std::cout << ("x: " + std::to_string(pose.x)) << std::endl;
        std::cout << ("y: " + std::to_string(pose.y)) << std::endl;
        std::cout << ("heading: " + std::to_string(pose.theta)) << std::endl;
        */
        
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
    // most points generated with https://5150vex.github.io/path.jerryio/
	setOrigin(); // sets the origin to where we place the bot

    if(selector::auton == 1){ // RED CLOSE (NOT WORKING WELL)
        /*
        chassis.moveToPoint(0, 0, 5000, false, 127, false);
        chassis.moveToPoint(0, -10, 5000, false, 127, false);
        wing.set_value(true);
        pros::delay(250);
        turnTo(-45);
        pros::delay(250);
        turnTo(0);
        pros::delay(250);
        chassis.moveToPoint(0, 0, 5000, false, 127, false);
        wing.set_value(false);

        setOrigin();

        chassis.moveToPoint(0, -6, 5000, false, 127, false);
        chassis.moveToPoint(0, 0, 5000, false, 127, false);
        */
        
        chassis.moveToPoint(0, 0, 5000, true, 127, false);
        outtake(127);
        chassis.moveToPoint(0, 6, 5000, true, 127, false); // 48
        outtake(0);
        chassis.moveToPoint(0, -24, 5000, false, 127, false); // 48
        turnTo(-90);

        setOrigin(false);
        
        intake(127);
        chassis.moveToPoint(12, 48, 5000, true, 127, false); // 48
        intake(0);
        chassis.moveToPoint(0, 0, 5000, false, 127, false); // 48
        
    }
    if(selector::auton == -2){ // BLUE FAR (SORTA WORKS)
        //pros::delay(500);
        chassis.moveToPoint(0, 0, 5000);
        wingSet(true);
        pros::delay(500);
        wingSet(false);
        turnTo(180);
        
        setOrigin();

        chassis.moveToPoint(0, -24, 5000);
        chassis.moveToPoint(24, -48, 5000);
        intake(127);
        chassis.moveToPoint(30, -54, 1200);
        chassis.moveToPoint(24, -48, 2000);
        turnTo(90);
        intake(0);
        //wingSet(true);

        // ram
        setOrigin();

        chassis.moveToPoint(0, -40, 1200); // -36
        outtake(127);
        chassis.moveToPoint(0, -24, 5000);
        outtake(0);
        //wingSet(false);
        chassis.moveToPoint(-36, 0, 5000);
        turnTo(179);

        // second intake
        setOrigin();
        
        chassis.moveToPoint(0, -20, 1200);
        chassis.moveToPoint(0, 0, 5000);
        chassis.moveToPoint(24, 24, 5000);
        chassis.moveToPoint(24, 36, 5000);
        turnTo(135);

        // match loader
        setOrigin();

        wingSet(true);
        chassis.moveToPoint(-12, -12, 5000);
        wingSet(false);
        chassis.moveToPoint(-24, -12, 5000);

    }
    if(selector::auton == -3){ // BLUE NO-OP (ACTUALLY NO OP)
    
    }
    if(selector::auton == 0){ //skills? (WHAT?)
        /* get silly ;P
            getSilly();
        */

        // move to match loader
        chassis.moveToPoint(13.000, 13.000, 5000);
        turnTo(180);

        // get ready to shoot
        setOrigin();
        wingSet(true);
        chassis.moveToPoint(0.000, -5.000, 1000);
        turnTo(-23);
        chassis.moveToPoint(1.000, -7.000, 1000);
        turnTo(-21);
        

        // shoot and reset
        //moveFor(puncher, 25000, 127); // 35000
        // moveFor(puncher, 1000, 127);
        
        wingSet(false);
        chassis.moveToPoint(0, 0, 5000);
        turnTo(0);

        // line up for opposite side
        setOrigin();
        chassis.moveToPoint(16.000, 16.000, 2000);
        turnTo(0);

        // go to other side for ram
        setOrigin();
        chassis.moveToPoint(0.000, 72.000, 5000);
        chassis.moveToPoint(-18.000, 90.000, 5000);
        chassis.moveToPoint(-76.000, 60.000, 5000);
        turnTo(0);

        // ram.
        int x = 0;
        setOrigin();
        wingSet(true);

        while(x <= 3){
            // back and forth
            chassis.moveToPoint(0, 360.000, 700);
            chassis.moveToPoint(0, 12.000, 700);
            turnTo(0);
            x++;
        }
        wingSet(false);
        
    }
}

void opcontrol() {

	while (true) {
        auto turn = controller.get_analog(ANALOG_RIGHT_X);
        auto power = controller.get_analog(ANALOG_LEFT_Y);

        // this turns the two stick into rotational and lateral movement
         left.move(power - (turn * turnPower()));
		right.move(power + (turn * turnPower()));

        // allows the voids to do their things (without this, the macros would not work)
		wingControl();
		sideHangControl();
		intakeControl();
		puncherControl();

        //screen();

        // saves a bit of memory
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