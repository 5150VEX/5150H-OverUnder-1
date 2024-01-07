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

pros::ADIDigitalOut wing ('g', false); // wing setup
//pros::ADIDigitalOut endgame ('h', false); // endgame setup
pros::ADIDigitalOut blocker ('h', false); // blocker setup

pros::Motor cata1(20, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES); // cata motor
pros::Motor cata2(10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES); // cata motor
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
pros::MotorGroup cata({cata1, cata2});

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

bool cataFlag = true;
bool cataState = false;

bool blockerFlag = true;
bool blockerState = false;

const double PI = 3.14159;

//==================================================//
// VOIDS				 	            			//
//==================================================//

//==================================================//
// v v v AUTO STUFF                           v v v //
void turnTo(float deg, float speed, int timeout){
    // using trig to turn a degree into a point that is far away from the bot in...
    // ... order to remove the need for odometry
	chassis.turnTo(sin(deg * (PI/180))*1000, cos(deg * (PI/180))*1000, timeout, false, speed);
}
void turnTo(float deg){
    // using trig to turn a degree into a point that is far away from the bot in...
    // ... order to remove the need for odometry
	chassis.turnTo(sin(deg * (PI/180))*1000, cos(deg * (PI/180))*1000, 1200, false, 127);
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

void setOrigin(){
    chassis.setPose(0, 0, 0); // sets the origin to where we place the bot
}
// ^ ^ ^ AUTO STUFF                           ^ ^ ^ //
//==================================================//

//==================================================//
// v v v CATA CONTROLS                        v v v //
void cataControlToggle(){
    // define the right two bumpers and triggers on the controller as booleans
	bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
	bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    
    // setting up a toggle for the wing
	if (cataFlag && r1) { // if r1 is pressed
		cataFlag = false;
        if (cataState){
            cataState = false;
		    cata.move(0);
        } else {
            cataState = true;
            cata.move(32);
        }
	}

    if (r2) { // if r2 is held
        cata.move(16);
    }

    // the magic of the toggle function
    // the wingFlag, once set to false, will only set to true again if the A button...
    // ... isn't pressed, this prevents repeated calls to open and close the wing
	if (!r1) {
		cataFlag = true;
	}
};
void cataControl(){
    // define the right two bumpers and triggers on the controller as booleans
	bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    
    if (r2) { // if r2 is held
        cata.move(127);
    }
    else {
        cata.move(0);
    }
};
// ^ ^ ^ CATA CONTROLS                        ^ ^ ^ //
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
    /*
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
    */
// ^ ^ ^ ENDGAME CONTROLS                     ^ ^ ^ //
//==================================================//

//==================================================//
// v v v BLOCKER CONTROLS                     v v v //
void blockerControl(){
	bool bb = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    // setting up a toggle for the hanger
	if (blockerFlag && bb) {
		blockerFlag = false;

        if (blockerState){
            blockerState = false;
		    blocker.set_value(false);
        } else {
            blockerState = true;
            blocker.set_value(true);
        }
	}

    // the blockerFlag, once set to false, will only set to true again if the left bumper...
    // ... isn't pressed, this prevents repeated calls to open and close the wing
	if (!bb) {
		blockerFlag = true;
	}
};
// ^ ^ ^ BLOCKER CONTROLS                     ^ ^ ^ //
//==================================================//

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        
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
    // points generated with https://5150vex.github.io/path.jerryio/
	setOrigin(); // sets the origin to where we place the bot

    if(selector::auton == 1){ // RED CLOSE
        //pros::delay(500);
        chassis.moveTo(0, 0, 5000);
        chassis.moveTo(14.868, 11.031, 5000);
        chassis.moveTo(24.257, 11.751, 1200); // RAM!!!!
        chassis.moveTo(14.868, 11.031, 2500);

        //chassis.moveTo(14.149, 3.717, 5000);
        
        wing.set_value(true);
        chassis.moveTo(-0.959, -4.317, 5000);
        //pros::delay(500);
        wing.set_value(false);
        
        chassis.moveTo(14.868, 11.031, 2500);
        chassis.moveTo(-0.959, -4.317, 5000);

        chassis.moveTo(-0.959, -34.712, 5000);
        
    }
    if(selector::auton == 2){ // RED FAR
        setOrigin();
        left = 127;
        right = 127;
        pros::delay(1000);
        left = 0;
        right = 0;
        pros::delay(1000);
        left = -127;
        right = -127;
        pros::delay(500);
        left = 0;
        right = 0;
    /*
        pros::delay(500);
        chassis.moveTo(0, 0, 5000);
        chassis.moveTo(-20.624, 26.859, 5000);
        chassis.moveTo(-48.442, 30.216, 1200);
        chassis.moveTo(-11.271, 30.216, 5000);
        //chassis.moveTo(-5.036, -32.375, 5000);
    */
    }
    if(selector::auton == 3){ // RED NO-OP

    }
    if(selector::auton == -1){ // BLUE CLOSE
        //pros::delay(500);
        // matchload into the goal
        chassis.moveTo(0, 0, 5000);
        chassis.moveTo(0, 48, 5000);
        turnTo(-90);
        outtake(127);
        pros::delay(500);
        outtake(0);

        setOrigin();
        
        // grab first middle triball
        chassis.moveTo(-12, 0, 5000);
        turnTo(90);
        intake(127);
        setOrigin();
        chassis.moveTo(0, 12, 5000);
        chassis.moveTo(0, 0, 5000);
        turnTo(180);
        
        setOrigin();

        // outtake first triball
        outtake(127);
        chassis.moveTo(24, 0, 2000);
        outtake(0);
        chassis.moveTo(0, 0, 5000);

        // match loader triball
        chassis.moveTo(-36, 24, 5000);
        wingSet(true);
        chassis.moveTo(-24, 36, 5000);
        wingSet(false);
        chassis.moveTo(20, 36, 5000);
    }
    if(selector::auton == -2){ // BLUE FAR
        //pros::delay(500);
        
    }
    if(selector::auton == -3){ // BLUE NO-OP
    
    }
    if(selector::auton == 0){ //skills?
        /* get silly ;P
        int* important = (int*)malloc;
        *important = 0;
        */

        // move to match loader
        chassis.moveTo(13.000, 13.000, 5000);
        turnTo(180);

        // get ready to shoot
        setOrigin();
        wingSet(true);
        chassis.moveTo(0.000, -6.000, 1000);
        turnTo(-15);

        // shoot and reset
        moveFor(cata, 35000, 96); // 35000
        // moveFor(cata, 1000, 127); // 35000
        
        wingSet(false);
        chassis.moveTo(0, 0, 5000);
        turnTo(0);

        // line up for opposite side
        setOrigin();
        chassis.moveTo(16.000, 16.000, 2000);
        turnTo(0);

        // go to other side for ram
        setOrigin();
        chassis.moveTo(0.000, 72.000, 5000);
        chassis.moveTo(-18.000, 90.000, 5000);
        chassis.moveTo(-76.000, 60.000, 5000);
        turnTo(0);

        // ram.
        int x = 0;
        setOrigin();
        wingSet(true);

        while(x <= 2){
            // back and forth
            chassis.moveTo(0, 360.000, 700);
            chassis.moveTo(0, 12.000, 700);
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
        left.move(power - turn);
		right.move(power + turn);

        // allows the voids to do their things (without this, the macros would not work)
		cataControl();
		wingControl();
		// endgameControl();
		intakeControl();
		blockerControl();

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