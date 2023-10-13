#include "main.h"
#include "hbot.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"


std::unique_ptr<Robot> robot = nullptr;
std::unique_ptr<Controller> controller = Controller::create(pros::Controller(pros::E_CONTROLLER_MASTER));

constexpr int32_t FLYWHEEL_NORMAL_RPM = 1900;
constexpr int32_t FLYWHEEL_ANGLECHG_RPM = 2000;
constexpr int32_t FLYWHEEL_OVERFILL_RPM = 1900;

void print_loop() {
	while (true) {
		auto pos = robot->controllers->odom->position();
		auto rpm = robot->flywheel->rpm();
 
		pros::lcd::print(0, "X: %f cm", pos.x);
		pros::lcd::print(1, "Y: %f cm", pos.y);
		pros::lcd::print(2, "Heading: %f degrees", pos.heading);
		pros::lcd::print(4, "Flywheel: %f RPM", rpm);

		pros::delay(20);
	}
}

void drive_loop() {
	robot->flywheel->move(FLYWHEEL_NORMAL_RPM);
	robot->flywheel->enable();
	robot->chassis->set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);

	int32_t flywheel_normal_rpm = FLYWHEEL_NORMAL_RPM;
	int32_t flywheel_anglechg_rpm = FLYWHEEL_ANGLECHG_RPM;
	bool toggle_overfill = false;

	while (true) {
        auto turn = controller->analog(ANALOG_RIGHT_X);
        auto power = controller->analog(ANALOG_LEFT_Y);
        robot->chassis->move_joystick(power, turn);

        if (controller->pressed(DIGITAL_R1) && 
			controller->pressed(DIGITAL_R2) && 
			controller->pressed(DIGITAL_L1) && 
			controller->pressed(DIGITAL_L2)) {
            robot->endgame->fire();
        }

        if(controller->pressed(DIGITAL_L1)) {
            robot->intake->move_voltage(12000);
        } else if (controller->pressed(DIGITAL_L2)) {
            robot->intake->move_voltage(-12000);
        } else {
            robot->intake->move_voltage(0);
        }
        
        if(controller->newly_pressed(DIGITAL_A)) {
            robot->flywheel->toggle();
			
        }

        if(controller->newly_pressed(DIGITAL_B)) {
            robot->anglechg->toggle();

            if(robot->anglechg->toggled()) {
                robot->flywheel->move(flywheel_anglechg_rpm);
            } else {
                robot->flywheel->move(flywheel_normal_rpm);
            }
        }

		if(controller->newly_pressed(DIGITAL_X)) {
			toggle_overfill = !toggle_overfill;

			if (toggle_overfill) {
				flywheel_anglechg_rpm = FLYWHEEL_OVERFILL_RPM;
			} else {
				flywheel_anglechg_rpm = FLYWHEEL_ANGLECHG_RPM;
			}
		}
		
        pros::delay(5);
    }
}

void fire_loop() {
	while (true) {
		if(controller->newly_pressed(DIGITAL_R1)) {
            if(controller->pressed(DIGITAL_R2)) {
                robot->indexer->index();
            } else {
				robot->indexer->repeat(3);
            }
        }
		pros::delay(5);
	}
}



void initialize() {
	pros::lcd::initialize();
	pros::Task pl(print_loop);
	pros::Task fl(fire_loop);


	auto controllers = Controllers::create(
		PID::create(600, 0, 62.5, 0, 0, 20),
		PID::create(400, 5, 45, 900, 0, 20),
		PID::create(0, 0, 0, 0, 0, 20),
		Odom::create(
			pros::Rotation(8), pros::Rotation(6, true), 
			2.75 * INCH_TO_CM, 5.25 * INCH_TO_CM)
	);

	auto chassis = Chassis::create(
		{19, -17, -18}, 
		{-12,11, 13},
		1, 1);

	auto intake = Intake::create(
		{-1});

	auto flywheel = Flywheel::create(
		{-10},
		pros::Rotation(9),
		PID::create(150, 0, 0, 895.0, 2.6, 10) // 
		);

	auto indexer = Indexer::create(
		pros::ADIDigitalOut('B'), 
		100, 200); //150, 265

	auto anglechg = Anglechg::create(
		pros::ADIDigitalOut('A'));

	auto endgame = Endgame::create(
		pros::ADIDigitalOut('C'));

	robot = Robot::create(
		std::move(chassis),
		std::move(controllers),
		std::move(intake),
		std::move(flywheel),
		std::move(indexer),
		std::move(anglechg),
		std::move(endgame));
}

void auto_solo() {
	// start flywheel
	robot->flywheel->enable();
	robot->flywheel->use_pidf();
	robot->flywheel->move(2400); 

	//get roller
	robot->drive_dist_timeout(-7.5, 1000, 5);
	robot->intake->move_for_voltage(250, 12000);
	robot->drive_dist(15);
	
	//shoot preload
	robot->turn_to_angle(-6);
	pros::delay(1500);
	robot->indexer->repeat(2, 300, 200);
	robot->flywheel->move(2200);
	

	//bump line of 3
	robot->turn_to_angle(-135);
	robot->chassis->set_velocity_percent(67);
	robot->drive_dist(-60);
	robot->chassis->set_velocity_percent(100);
	robot->intake->move_voltage(12000);

	//intake of 3 
	robot->chassis->set_voltage_percent(50);
	//robot->drive_dist(-75);
	robot->drive_to_point_noturn(114.6, 96.2, true);
	
	robot->chassis->set_voltage_percent(100);

	//shoot line of 3
	robot->turn_to_angle(-37.75);
	robot->indexer->repeat(3, 300, 200);

	//set for far 3 shots
	robot->flywheel->move(2400);

	//intake line of 3
	robot->turn_to_angle(-140);
	robot->chassis->set_voltage_percent(90);
	robot->drive_dist(-185);

	//roller
	robot->turn_to_angle(-90);
	robot->drive_dist_timeout(-7.5, 1000, 5);
	robot->intake->move_for_voltage(250, 12000);
	robot->drive_dist(15);

	//shoot 3
	robot->turn_to_angle(-78.8);
	robot->indexer->repeat(3, 325, 100);
}

void auto_left() {
	// start flywheel


	robot->flywheel->enable();
	robot->flywheel->use_pidf();
	robot->flywheel->move(2340); 

	//get roller
	robot->drive_dist_timeout(-7.5, 1000, 5);
	robot->intake->move_for_voltage(250, 12000);
	robot->drive_dist(15);
	
	//shoot preload
	robot->turn_to_angle(-5.75);
	pros::delay(1250);
	robot->indexer->repeat(2, 300, 200);
	robot->flywheel->move(2200);
	
	//bump line of 3
	robot->turn_to_angle(-135);
	robot->chassis->set_velocity_percent(45);
	robot->drive_dist(-58);
	robot->chassis->set_velocity_percent(100);
	robot->intake->move_voltage(12000);

	//intake of 3 
	robot->chassis->set_voltage_percent(50);
	//robot->drive_dist(-75);

	robot->drive_to_point_noturn(101.25, 84.25, true);
	
	robot->chassis->set_voltage_percent(100);

	robot->drive_to_point_noturn(87.25, 75.3);

	//shoot line of 3
	robot->turn_to_angle(-30.5);
	robot->indexer->repeat(3, 300, 200);

	robot->flywheel->move(2220);

	//boomerang

	robot->turn_to_angle(-60);
	robot->drive_dist(-42, 10, 850); //-38

	robot->drive_to_point_noturn(87.25, 75.3);

	//shoot line of 3
	robot->turn_to_angle(-31.5);
	robot->indexer->repeat(3, 300, 200);

}

void auto_right() {
	// start flywheel
	robot->flywheel->enable();
	robot->flywheel->use_pidf();
	robot->flywheel->move(2440);
	
	// drive to roller
	robot->drive_dist_timeout(-55, 750, true);
	// turn to face roller
	robot->turn_to_angle(90);
	// drive into roller
	robot->drive_dist_timeout(-10, 1000, 5);
	// spin for 250ms
	robot->intake->move_for_voltage(275, 12000);
	// drive away
	robot->drive_dist(5);

	// turn to goal
	robot->turn_to_angle(107.5);
	pros::delay(350);
	// shoot 2 preloads
	robot->indexer->repeat(2, 500, 200);
	// prepare lower flywheel velocity for next shots
	robot->flywheel->move(2160);
		
	// start intake
	robot->intake->move_voltage(12000);

	// turn to 3line
	robot->turn_to_angle_timeout(228, 1000, 2.5);
	
	// drive and intake 3 line
	robot->chassis->set_voltage_percent(70);

	robot->drive_to_point(51, 114.5, true);

	robot->chassis->set_voltage_percent(100);

	robot->turn_to_angle(140);
	robot->indexer->repeat(3, 400, 200);
	robot->flywheel->move(2160);
	
	// drive into boomerang
	robot->drive_dist_timeout(-26, 1000, 7.5);

	// drive back
	robot->drive_to_point(51, 114.5);
	// turn to shoot
	robot->turn_to_angle(141);
	
	// shoot
	robot->indexer->repeat(3, 300, 200);

	
}

void auto_skills() {
	
	robot->flywheel->enable();
	robot->flywheel->move(1800);
	
	
	pros::delay(2750);

	robot->indexer->repeat(9, 1000);

	// heading = 23.5
	

	robot->drive_to_point(38.5, -1.5);

	robot->intake->move_voltage(12000);

	robot->turn_to_angle(15);

	robot->chassis->set_voltage_percent(50);
	robot->drive_to_point(-129, -52, true);
	robot->chassis->set_voltage_percent(100);

	robot->turn_to_angle(0);
	robot->drive_dist(-18.25, 5);
	pros::delay(750);
	robot->drive_dist(30);

	robot->turn_to_angle(-88);
	robot->drive_dist(-37.5, 5);
	pros::delay(750);
	robot->drive_to_point(-112.5, -95);
	robot->indexer->repeat(1, 1000, 100);
	robot->turn_to_angle(-218);
	robot->chassis->set_voltage_percent(50);
	robot->drive_to_point(12, -199, true);
	
	robot->flywheel->move(1950);
	robot->turn_to_angle(-126);
	robot->indexer->repeat(3, 1000, 100);

	robot->turn_to_angle(-218);
	robot->drive_dist(16.75);
	robot->turn_to_angle(10);
	robot->chassis->set_voltage_percent(35);
	robot->drive_dist(-130);

	robot->chassis->set_voltage_percent(80);
	robot->flywheel->move(2000);

	robot->turn_to_angle(-307);

	robot->drive_to_point(-80.5, -138);
	robot->turn_to_angle(-93);
	robot->indexer->repeat(3, 1000, 100);

	robot->chassis->set_voltage_percent(40);

	robot->turn_to_angle(-135.75);

	robot->drive_dist(-30);

	robot->turn_to_angle(-230);

	//robot->drive_to_point(-68.5, -98.5);

	robot->flywheel->move(2100);
	robot->drive_to_point(51.5, -174, true);
	robot->turn_to_angle(-138);
	robot->indexer->repeat(3, 1000, 100);

	robot->chassis->set_voltage_percent(100);	
	robot->drive_to_point(79.5, -156.25, true);
	robot->turn_to_angle(-230);
	
	//robot->indexer->repeat(3, 1000, 100);

}

void auto_skills_old() {
	/*
	robot->flywheel->enable();
	robot->flywheel->move(2000);
	pros::delay(3000);

	robot->indexer->repeat(9, 1000);*/

	robot->drive_to_point(147.5, 0);

	robot->drive_to_point(151, -238.25, true);
	
}

void autonomous() {
	pros::Task auto_(auto_left);
	fire_loop();
}

void opcontrol() {
	pros::Task drive(drive_loop);
	fire_loop();
}
