#pragma once
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <mutex>
#include <numeric>

#define LOG_DEBUG false

#ifdef LOG_DEBUG
	#define LOG(message) std::cout << message
#else
	#define LOG(message)
#endif

constexpr double INCH_TO_CM = 2.54;
constexpr double RADIAN_TO_DEGREE = (180.0 / M_PI);
constexpr double DEGREE_TO_RADIAN = (M_PI / 180.0);

class PID {
	double kp;
    double ki;
	double kd;
	double kb;
	double kf;
	unsigned long interval;

	double setpoint = 0;
    double reading = 0;
    double error = 0;
    double output = 0;

	double p = 0;
	double i = 0;
	double d = 0;
	double b = 0;
	double f = 0;

    double prev_reading = 0;
    double prev_error = 0;
	unsigned long prev_time = 0;
public:
	PID(double ikp, double iki, double ikd, double ikb, double ikf, unsigned long iinterval) :
	kp(ikp), ki(iki), kd(ikd), kb(ikb), kf(ikf), interval(iinterval) {
        double interval_in_sec = interval / 1000.0;
        ki *= interval_in_sec;
        kd /= interval_in_sec;
	}

	inline double step(double new_reading) {
        reading = new_reading;
		error = setpoint - reading;
        unsigned long time = pros::millis();

        if (time - prev_time < interval) {
            return output;
        }

		p = kp * error;
		i += ki * error;
        d = kd * (reading - prev_reading) * -1;
        b = std::copysign(kb, reading);
        f = kf * setpoint;

        // reset integral on zero cross
        if(std::signbit(error) != std::signbit(prev_error)) {
            i = 0;
        }

		output = std::clamp(p + i + d + b + f, -12000.0, 12000.0);

        prev_error = error;
        prev_reading = reading;

		return output;
	}

    inline void target(double new_setpoint) {
        setpoint = new_setpoint;
        error = new_setpoint;

        p = 0;
        i = 0;
        d = 0;
        b = 0;
        f = 0;
    }

    inline double get_setpoint() {
        return setpoint;
    }

    inline double get_reading() {
        return reading;
    }

    inline double get_error() {
        return error;
    }

    inline double get_output() {
        return output;
    }

    inline unsigned long get_interval() {
        return interval;
    }

	inline double get_p() {
		return p;
	}
	
	inline double get_i() {
		return i;
	}

	inline double get_d() {
		return d;
	}

	inline static std::unique_ptr<PID> create(double ikp, double iki, double ikd, double ikb, double ikf, unsigned long iinterval) {
		return std::make_unique<PID>(ikp, iki, ikd, ikb, ikf, iinterval);
	}
};

struct Position {
    double x;
    double y;
    double theta;
    double heading;
};

class Odom {
	pros::Rotation left;
	pros::Rotation right;

	pros::Mutex lock;
    pros::Task thread;

	const double diameter;
	const double trackwidth;

    double prev_l = 0;
    double prev_r = 0;

    double global_x = 0;
    double global_y = 0;
    double global_theta = 0;

    void loop() {
        while (true) {
            std::unique_lock<pros::Mutex> guard(lock);
            
            double current_l = left.get_position() / 36000.0 * diameter * M_PI;
            double current_r = right.get_position() / 36000.0 * diameter * M_PI;
            
            double delta_l = current_l - prev_l;
            double delta_r = current_r - prev_r;

            double local_theta = (delta_l - delta_r) / trackwidth;
            double local_x = (delta_l + delta_r) / 2.0;
            double local_y = 0;

            global_theta = global_theta + local_theta;

            double sin_theta = std::sin(global_theta);
            double cos_theta = std::cos(global_theta);

            global_x += (local_x * cos_theta - local_y * sin_theta);
            global_y += (local_y * cos_theta + local_x * sin_theta);

            prev_l = current_l;
            prev_r = current_r;

            guard.unlock();
            pros::delay(20);
        }
    }
public:
	Odom(pros::Rotation ileft, pros::Rotation iright, double idiameter, double itrackwidth) : 
    left(ileft), right(iright), diameter(idiameter), trackwidth(itrackwidth), thread([&]{ this->loop(); }) {
		left.set_position(0);
		right.set_position(0);
	}
	
	inline double heading(bool radians = false) {
		std::lock_guard<pros::Mutex> guard(lock);
		double wrapped = std::fmod(global_theta, 360 * DEGREE_TO_RADIAN);

		if (radians) {
			return wrapped;
		} else {
			return wrapped * RADIAN_TO_DEGREE;
		}
	}
	inline double raw_heading(bool radians = false) {
		std::lock_guard<pros::Mutex> guard(lock);
        if (radians) {
            return global_theta;
        } else {
            return global_theta * RADIAN_TO_DEGREE;
        }
	}

    inline double forward() {
        double l = left.get_position() / 36000.0 * diameter * M_PI;
        double r = right.get_position() / 36000.0 * diameter * M_PI;
        return (l + r) / 2.0;
    }

    inline double x() {
        std::lock_guard<pros::Mutex> guard(lock);
        return global_x;
    }

    inline double y() {
        std::lock_guard<pros::Mutex> guard(lock);
        return global_y;
    }

    inline Position position() {
        std::lock_guard<pros::Mutex> guard(lock);
		double wrapped_heading = std::fmod(global_theta, 360 * DEGREE_TO_RADIAN);

        return Position {
            global_x,
            global_y,
            wrapped_heading,
            wrapped_heading * RADIAN_TO_DEGREE
        };
    }

	inline static std::unique_ptr<Odom> create(pros::Rotation ileft, pros::Rotation iright, double idiameter, double itrackwidth) {
		return std::make_unique<Odom>(ileft, iright, idiameter, itrackwidth);
	}
};

class Controllers {
public:
	std::unique_ptr<PID> drive;
	std::unique_ptr<PID> turn;
	std::unique_ptr<PID> angle;
	std::unique_ptr<Odom> odom;

	Controllers(std::unique_ptr<PID> idrive, std::unique_ptr<PID> iturn, std::unique_ptr<PID> iangle, std::unique_ptr<Odom> iodom) :
	drive(std::move(idrive)), turn(std::move(iturn)), angle(std::move(iangle)), odom(std::move(iodom)) {
	}

	inline static std::unique_ptr<Controllers> create(std::unique_ptr<PID> idrive, std::unique_ptr<PID> iturn, std::unique_ptr<PID> iangle, std::unique_ptr<Odom> iodom) {
		return std::make_unique<Controllers>(std::move(idrive), std::move(iturn), std::move(iangle), std::move(iodom));
	}
};

class Chassis {
	pros::MotorGroup left;
	pros::MotorGroup right;
	const int32_t exp_power;
	const int32_t exp_turn;

	double voltage_percent = 1;
	double velocity_percent = 1;

	double voltage_max = 12000;
	double velocity_max = 200;

	inline double exp_drive(int32_t power, int32_t exponent) {
		auto value = std::pow((power / 127.0), exponent) / static_cast<double>(std::pow(1, exponent - 1));
    	return value  * 127;
	}

public:
	Chassis(std::initializer_list<int8_t> ileft, std::initializer_list<int8_t> iright, double iexp_power = 1, double iexp_turn = 1) : 
	left(ileft), right(iright), exp_power(iexp_power), exp_turn(iexp_turn) {
	};

	inline void set_voltage_max(double max) {
		voltage_max = max;
	}

	inline void set_velocity_max(double max) {
		velocity_max = max;
	}

	inline void set_voltage_percent(double percent) {
		percent /= 100.0;
		voltage_percent = std::clamp(percent, 0.0, 1.0);
	}

	inline void set_velocity_percent(double percent) {
		percent /= 100.0;
		velocity_percent = std::clamp(percent, 0.0, 1.0);
	}

	inline void move_voltage(int32_t power, int32_t turn) {
		left.move_voltage((power + turn) * voltage_percent);
		right.move_voltage((power - turn) * voltage_percent);
	}

	inline void move_velocity(int32_t power, int32_t turn) {
		left.move_velocity(std::clamp((power + turn) * velocity_percent, -velocity_max, velocity_max));
		right.move_velocity(std::clamp((power - turn) * velocity_percent, -velocity_max, velocity_max));
	}

	inline void drive_voltage(int32_t voltage) {
		move_voltage(voltage, 0);
	}

	inline void turn_voltage(int32_t voltage) {
		move_voltage(0, voltage);
	}

	inline void drive_velocity(int32_t velocity) {
		move_velocity(velocity, 0);
	}

	inline void turn_velocity(int32_t velocity) {
		move_velocity(0, velocity);
	}

	inline void move_joystick(int32_t power, int32_t turn) {
		power = exp_drive(power, exp_power);
		turn = exp_drive(turn, exp_turn);
		left.move(power + turn);
		right.move(power - turn);
	}

	inline void stop() {
		move_voltage(0,0);
	}

	inline void set_brake_mode(pros::motor_brake_mode_e_t mode) {
		left.set_brake_modes(mode);
		right.set_brake_modes(mode);
	}

	inline static std::unique_ptr<Chassis> create(std::initializer_list<int8_t> ileft, std::initializer_list<int8_t> iright, double iexp_power = 1, double iexp_turn = 1) {
		return std::make_unique<Chassis>(ileft, iright, iexp_power, iexp_turn);
	};
};

class Intake {
	pros::MotorGroup motors;
public:
	Intake(std::initializer_list<int8_t> imotors) : 
	motors(imotors) {
	}

	inline void move_voltage(int32_t voltage) {
		motors.move_voltage(voltage);
	}

	inline void move_velocity(int32_t velocity) {
		motors.move_velocity(velocity);
	}

	inline void move_for_voltage(unsigned long time, int32_t voltage) {
		move_voltage(voltage);
		pros::delay(time);
		move_voltage(0);
	}
	
	inline void move_for_velocity(unsigned long time, int32_t velocity) {
		move_voltage(velocity);
		pros::delay(time);
		move_voltage(0);
	}

	inline static std::unique_ptr<Intake> create(std::initializer_list<int8_t> imotors) {
		return std::make_unique<Intake>(imotors);
	}
};

class Flywheel {
	pros::MotorGroup motors;
	pros::Rotation sensor;
	std::unique_ptr<PID> controller;

    pros::Task thread;
    pros::Mutex lock;

	bool enabled = false;
	bool bangbang = false;

	double prev_pos = 0;
	double prev_vel = 0;

    void loop() {
        unsigned long interval = controller->get_interval();
		auto interval_in_sec = interval / 1000.0;
		const double alpha = 0.1;
		double filtered = 0;

        while (true) {
            std::unique_lock<pros::Mutex> guard(lock);
        
            if (enabled) {
				if (bangbang) {
					double reading = (sensor.get_velocity() / 360.0 * 60.0);
					double setpoint = controller->get_setpoint();
					double diff = setpoint - reading;

				} else {
					double pos = sensor.get_position();
					double delta_pos = (pos - prev_pos) / interval_in_sec;

					double vel = (sensor.get_velocity() / 360.0 * 60.0);
					double accel = (vel - prev_vel) / interval_in_sec;

					auto mreading = motors.get_actual_velocities().at(0) * 18.0;
					filtered = (alpha * mreading) + (1.0 - alpha) * filtered;

                	double voltage = std::clamp(controller->step(filtered), 0.0, 12000.0);
					pros::lcd::print(6, "MV: %f", voltage);
					//std::cout << controller->get_setpoint() << "," << voltage << "," << vel << "," << accel << "\n";
					
					prev_pos = pos;
					prev_vel = vel;
					
					motors.move_voltage(voltage);
				}
            } else {
                motors.move_voltage(0);
            }
            
            guard.unlock();
            pros::delay(interval);
        }
    }

public:
	Flywheel(std::initializer_list<int8_t> imotors, pros::Rotation isensor, std::unique_ptr<PID> icontroller) : 
	motors(imotors), sensor(isensor), controller(std::move(icontroller)), thread([&] { this->loop(); }) {
        sensor.set_data_rate(10);
		
	}

	inline void move(double rpm) {
        std::lock_guard<pros::Mutex> guard(lock);
        controller->target(rpm);
	}

	inline void enable() {
        std::lock_guard<pros::Mutex> guard(lock);
		enabled = true;
	}

	inline void disable() {
        std::lock_guard<pros::Mutex> guard(lock);
		enabled = false;
	}

	inline void toggle() {
        std::lock_guard<pros::Mutex> guard(lock);
		enabled = !enabled;
	}

    inline bool toggled() {
        std::lock_guard<pros::Mutex> guard(lock);
        return enabled;
    }
    
    inline double rpm() {
        return sensor.get_velocity() / 360.0 * 60.0;
    }

	inline void use_bangbang() {
		bangbang = true;
	}

	inline void use_pidf() {
		bangbang = false;
	}

	inline double get_reading() {
		std::lock_guard<pros::Mutex> guard(lock);
		return sensor.get_velocity() / 360.0 * 60.0;
	}

	inline static std::unique_ptr<Flywheel> create(std::initializer_list<int8_t> imotors, pros::Rotation isensor, std::unique_ptr<PID> icontroller) {
		return std::make_unique<Flywheel>(imotors, isensor, std::move(icontroller));
	}
};

class Indexer {
	pros::ADIDigitalOut piston;
	const unsigned long delay;
	const unsigned long interval;
public:
	Indexer(pros::ADIDigitalOut ipiston, unsigned long idelay, unsigned long iinterval) :
	piston(ipiston), delay(idelay), interval(iinterval) {
	}

	inline void extend() {
		piston.set_value(true);
	}

	inline void retract() {
		piston.set_value(false);
	}

	inline void index() {
		extend();
		pros::delay(delay);
		retract();
	}

	inline void index(unsigned long delay) {
		extend();
		pros::delay(delay);
		retract();
	}

	inline void repeat(int times) {
		for(int i = 0; i < times - 1; i++) {
			index();
			pros::delay(interval);
		}
		index();
	}

	inline void repeat(int times, unsigned long interval) {
		for(int i = 0; i < times - 1; i++) {
			index();
			pros::delay(interval);
		}
		index();
	}

	inline void repeat(int times, unsigned long interval, unsigned long delay) {
		for(int i = 0; i < times - 1; i++) {
			index(delay);
			pros::delay(interval);
			std::cout << "waited interval\n";
		}
		index(delay);
	}

	inline static std::unique_ptr<Indexer> create(pros::ADIDigitalOut ipiston, unsigned long idelay, unsigned long iinterval) {
		return std::make_unique<Indexer>(ipiston, idelay, iinterval);
	}
};

class Anglechg {
	pros::ADIDigitalOut piston;
	bool extended = false;
public:
	Anglechg(pros::ADIDigitalOut ipiston) :
	piston(ipiston) {
	}

	inline void extend() {
		extended = true;
		piston.set_value(true);
	}

	inline void retract() {
		extended = false;
		piston.set_value(false);
	}

	inline void toggle() {
		extended = !extended;
		piston.set_value(extended);
	}

	inline bool toggled() {
		return extended;
	}

	inline static std::unique_ptr<Anglechg> create(pros::ADIDigitalOut ipiston) {
		return std::make_unique<Anglechg>(ipiston);
	}
};

class Endgame {
	pros::ADIDigitalOut piston;
public:
	Endgame(pros::ADIDigitalOut ipiston) :
	piston(ipiston) {
	}

	inline void fire() {
		piston.set_value(true);
	}

	inline static std::unique_ptr<Endgame> create(pros::ADIDigitalOut ipiston) {
		return std::make_unique<Endgame>(ipiston);
	}
};

class Robot {
	inline double constrain_angle_180(double degrees) {
		degrees = std::fmod(degrees, 360); 
		degrees = std::fmod((degrees + 360), 360);  
		if (degrees > 180) { degrees -= 360; }
		
		return degrees;
	}

	inline double calc_angle_to_point(double x, double y, bool reverse = false) {
		double target_x = x;
		double target_y = y;

		double current_x = controllers->odom->x();
		double current_y = controllers->odom->y();

		double angle = std::atan2(target_y - current_y, target_x - current_x) * RADIAN_TO_DEGREE;
		
		if (reverse) { angle += 180; };

		return constrain_angle_180(angle);
	}

	inline double calc_dist_to_point(double x, double y, bool reverse) {
		double target_x = x;
		double target_y = y;

		double current_x = controllers->odom->x();
		double current_y = controllers->odom->y();

		double dist = std::hypot(target_x - current_x, target_y - current_y);
		if (reverse) { dist = -dist; };

		return dist;
	}
public:
	std::unique_ptr<Chassis> chassis;
	std::unique_ptr<Controllers> controllers;
	std::unique_ptr<Intake> intake;
	std::unique_ptr<Flywheel> flywheel;
	std::unique_ptr<Indexer> indexer;
	std::unique_ptr<Anglechg> anglechg;
	std::unique_ptr<Endgame> endgame;

	Robot(std::unique_ptr<Chassis> ichassis, 
	std::unique_ptr<Controllers> icontrollers, 
	std::unique_ptr<Intake> iintake,
	std::unique_ptr<Flywheel> iflywheel,
	std::unique_ptr<Indexer> iindexer,
	std::unique_ptr<Anglechg> ianglechg,
	std::unique_ptr<Endgame> iendgame) :
	chassis(std::move(ichassis)), 
	controllers(std::move(icontrollers)),
	intake(std::move(iintake)),
	flywheel(std::move(iflywheel)),
	indexer(std::move(iindexer)),
	anglechg(std::move(ianglechg)),
	endgame(std::move(iendgame)) {
	}

	inline void drive_dist_timeout(double cm, unsigned long timeout, double error_threshold = 2, unsigned long required_time = 250) {
		LOG("[PID] Driving " << cm << " cm\n");

		double straight = controllers->odom->heading();
        double offset = controllers->odom->forward();
        controllers->drive->target(cm);
        unsigned long interval = controllers->drive->get_interval();
        
        bool settling = false;
        unsigned long settled_time = 0;
		unsigned long start_time = pros::millis();

        while (true) {
            if (!settling && std::abs(controllers->drive->get_error()) < error_threshold) {
                settled_time = pros::millis();
                settling = true;
            }

            if(settling) {
                if (std::abs(controllers->drive->get_error()) < error_threshold) {
                    unsigned long current_time = pros::millis();
                    unsigned long diff_time = current_time - settled_time;
                    if (diff_time > required_time) {
                        break;
                    }
                } else {
                    settling = false;
                }
            }

			unsigned long current_time = pros::millis();
			if(current_time - start_time > timeout) {
				break;
			}

            double dist = controllers->odom->forward() - offset;
			double drift = constrain_angle_180(controllers->odom->heading() - straight);

            double power = controllers->drive->step(dist);
			double turn = controllers->angle->step(drift);


			//std::cout << controllers->drive->get_setpoint() << "," << controllers->drive->get_reading() << "\n";

            chassis->move_voltage(power, turn);
            pros::delay(interval);
        }   
        
        chassis->stop();
        LOG("[PID] Finished movement at " << controllers->drive->get_error() << " cm error.\n");
	}

	inline void drive_dist(double cm, double error_threshold = 2, unsigned long required_time = 100) {
        drive_dist_timeout(cm, LONG_MAX, error_threshold, required_time);
    }

	inline void turn_angle_timeout(double degrees, unsigned long timeout, double error_threshold = 2, unsigned long required_time = 100) {
        LOG("[PID] Turning " << degrees << " degrees\n");

        double offset = controllers->odom->raw_heading();
        controllers->turn->target(degrees);

        unsigned long interval = controllers->turn->get_interval();
        
        bool settling_err = false;
        unsigned long err_time = 0;
		unsigned long start_time = pros::millis();

        while (true) {

            if (!settling_err && std::abs(controllers->turn->get_error()) < error_threshold) {
                err_time = pros::millis();
                settling_err = true;
            }

            if(settling_err) {
                if (std::abs(controllers->turn->get_error()) < error_threshold) {
                    unsigned long current_time = pros::millis();
                    unsigned long diff_time = current_time - err_time;
                    if (diff_time > required_time) {
                        break;
                    } 
				} else {
                    settling_err = false;
                }
            }

			unsigned long current_time = pros::millis();
			auto time_diff = current_time - start_time;
			if (time_diff > timeout) {
				break;
			} else if (timeout != LONG_MAX) {
				//std::cout << "time diff: " << time_diff << ", timeout: " << timeout << "\n";
			}

            double turn = controllers->odom->raw_heading() - offset;
            double voltage = controllers->turn->step(turn);

			//std::cout << controllers->turn->get_setpoint() << "," << controllers->turn->get_reading() << "\n";

            chassis->turn_voltage(voltage);
            pros::delay(interval);
        }   
        
        chassis->stop();
        LOG("[PID] Finished movement at " << controllers->turn->get_error() << " degrees error.\n\n");
    }

	inline void turn_angle(double degrees, double error_threshold = 2, unsigned long required_time = 250) {
        turn_angle_timeout(degrees, LONG_MAX, error_threshold, required_time);
    }

	inline void turn_to_angle(double degrees, double error_threshold = 2, unsigned long required_time = 250) {
		turn_to_angle_timeout(degrees, LONG_MAX, error_threshold, required_time);
	}

	inline void turn_to_angle_timeout(double degrees, unsigned long ms, double error_threshold = 2, unsigned long required_time = 250) {
		double heading = controllers->odom->heading();
		double diff = constrain_angle_180(degrees - heading);
		LOG("[PID] Turning to angle " << degrees << "\n");
		turn_angle_timeout(diff, ms, error_threshold, required_time);
	}

	inline void turn_to_point(double x, double y, bool reverse = false) {
		double angle = calc_angle_to_point(x, y, reverse);
		LOG("[Odom] Turning to point (" << x << ", " << y << ")\n");
		LOG("[Odom] Calculated angle " << angle << " degrees to point\n");
		turn_to_angle(angle);
	}

	inline void drive_to_point(double x, double y, bool reverse = false) {
		LOG("[Odom] Beginning movement to point (" << x << ", " << y << ")\n");
		turn_to_point(x, y, reverse);
		double dist = calc_dist_to_point(x, y, reverse);
		LOG("[Odom] Driving to point (" << x << ", " << y << ")\n");
		LOG("[Odom] Calculated dist " << dist << " cm to point\n");
		drive_dist(dist);
	}

	inline void drive_to_point_noturn(double x, double y, bool reverse = false) {
		double dist = calc_dist_to_point(x, y, reverse);
		LOG("[Odom] Driving to point (" << x << ", " << y << ")\n");
		LOG("[Odom] Calculated dist " << dist << " cm to point\n");
		drive_dist(dist);
	}
	inline static std::unique_ptr<Robot> create(
		std::unique_ptr<Chassis> ichassis, 
		std::unique_ptr<Controllers> icontrollers, 
		std::unique_ptr<Intake> iintake,
		std::unique_ptr<Flywheel> iflywheel,
		std::unique_ptr<Indexer> iindexer,
		std::unique_ptr<Anglechg> ianglechg,
		std::unique_ptr<Endgame> iendgame) {
		
		return std::make_unique<Robot>(
			std::move(ichassis), 
			std::move(icontrollers),
			std::move(iintake),
			std::move(iflywheel),
			std::move(iindexer),
			std::move(ianglechg),
			std::move(iendgame)
		);
	}
};

class Controller {
	pros::Controller controller;
public:
	Controller(pros::Controller icontroller) :
	controller(icontroller) {
	}

	inline bool pressed(pros::controller_digital_e_t button) {
		return controller.get_digital(button);
	}

	inline bool newly_pressed(pros::controller_digital_e_t button) {
		return controller.get_digital_new_press(button);
	}

	inline int32_t analog(pros::controller_analog_e_t channel) {
		return controller.get_analog(channel);
	}

	inline static std::unique_ptr<Controller> create(pros::Controller icontroller) {
		return std::make_unique<Controller>(icontroller);
	}
};