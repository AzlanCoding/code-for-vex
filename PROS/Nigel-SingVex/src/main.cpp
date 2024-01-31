#define _USE_MATH_DEFINES
#undef __STRICT_ANSI__
#include "main.h"
#include <numeric>
using namespace pros;

Motor LF(11);
Motor LM(12);
Motor LB(13);
Motor_Group left ({LF,LM,LB});
Motor RF(16,true);
Motor RM(17,true);
Motor RB(18,true);
Motor_Group right ({RF,RM,RB});
Motor PL(5,E_MOTOR_GEAR_GREEN);
Motor PR(10,E_MOTOR_GEAR_GREEN,true);

Controller H(E_CONTROLLER_MASTER);

Imu imu(14);

bool prevTurningIsClockwise = true;
std::string auton= "left";

double avg(const std::vector<double>& v) {
    if (v.empty()) {
        return 0.0;
    }
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    return sum / v.size();
}


double L_or_R(double actual, double target){
	double clockwise;
	double antiClockwise;
	if (actual > target){
		antiClockwise = actual - target;
		clockwise = 360 - antiClockwise;
	}
	else if (target > actual){
		clockwise = target - actual;
		antiClockwise = 360 - clockwise;
	}
	else{
		return 0;
	}
	if (clockwise < antiClockwise){
		prevTurningIsClockwise = true;
		return clockwise;
	}
	else if (antiClockwise < clockwise){
		prevTurningIsClockwise = false;
		return antiClockwise*-1;
	}
	else {//equal
	    if (prevTurningIsClockwise == true){
			return clockwise;
		}
		else{//prevTurningIsClockwise == false
		    return antiClockwise*-1;
		}
	}
}




void FWD(double target, double const speedConst = 0,bool stop=true, int timeout = 3000){
	motor_brake_mode_e_t lbm;
	motor_brake_mode_e_t rbm;
	left.tare_position();
	right.tare_position();
	if (stop){
		motor_brake_mode_e_t lbm = left.get_brake_modes()[0];//lbm = left brake mode
		motor_brake_mode_e_t rbm = right.get_brake_modes()[0];
		left.set_brake_modes(E_MOTOR_BRAKE_HOLD);
		right.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	}
	double dist = ((avg(left.get_positions()) + avg(right.get_positions()))/2);
	double err = target - dist;
	double speed;
	if (speedConst == 0){
		speed = (1-(dist/target))+0.4;// min speed is 20% (2.4 Volts)
	}
	else{
		speed = speedConst;
	}
	double t = pros::c::millis() + timeout;
	while (pros::c::millis() <= t){
		//std::cout << "[" << left.get_positions()[0] << "  " << left.get_positions()[1] << "  " << left.get_positions()[2] << "  " << right.get_positions()[0] << "  " << right.get_positions()[1] << "  " << right.get_positions()[2] << "]\n";
		std::cout << err <<"\n";
		dist = (((avg(left.get_positions())) + avg(right.get_positions()))/2)*3.25*M_PI;
		err = target - dist;
		if (speedConst == 0){
		speed = (1-(dist/target))+0.4;// min speed is 20% (2.4 Volts)
		}
		else{
			speed = speedConst;
		}
		if (err >= -0.5 && err <= 0.5){
			break;
		}
		if (err > 0){
		left.move(127*speed);
		right.move(127*speed);
		}
		else if (err < 0){
			left.move(127*0.2*-1);
			right.move(127*0.2*-1);
		}
		/*if (err > 0){
			left.move(127);
			right.move(127);
		}
		else if (err < 0){
			left.move(-127);
			right.move(-127);
		}*/
	}
	if (stop){
		std::cout << "stop!\n"; 
		left.brake();
		right.brake();
		t = pros::c::millis() + 1000;
		while (avg(left.get_actual_velocities()) != 0 || avg(right.get_actual_velocities()) != 0 || pros::c::millis() <= t){}//Wait for motor to stop or 1 second to pass
		left.set_brake_modes(lbm);
		right.set_brake_modes(rbm);
	}
}


void head(double target, double const speedConst = 0, bool stop=true, int timeout = 3000){
	motor_brake_mode_e_t lbm;
	motor_brake_mode_e_t rbm;
	double t = pros::c::millis() + timeout;
	if (stop){
		motor_brake_mode_e_t lbm = left.get_brake_modes()[0];//lbm = left brake mode
		motor_brake_mode_e_t rbm = right.get_brake_modes()[0];
		left.set_brake_modes(E_MOTOR_BRAKE_HOLD);
		right.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	}
	double err = L_or_R(imu.get_heading(),target);
	const double initialDist = abs(err);
	double speed;
	if (speedConst == 0){
		speed = (abs(err)/initialDist)+0.2;// min speed is 20% (2.4 Volts)
	}
	else{
		speed = speedConst;
	}
	while ((err >= 2 && err <= -2) || pros::c::millis() <= t){
		err = L_or_R(imu.get_heading(),target);
		if (speedConst == 0){
		speed = (abs(err)/initialDist)+0.2;// min speed is 20% (2.4 Volts)
	}
	else{
		speed = speedConst;
	}
		if (err >= 1){
			left.move(127*speed);
			right.move(127*speed*-1);
		}
		else if (err <= -1){
			left.move(127*speed*-1);
			right.move(127*speed);
		}
		else{
			break;
		}
	}
	if (stop){
		left.brake();
		right.brake();
		t = pros::c::millis() + 1000;
		while (avg(left.get_actual_velocities()) != 0 || avg(right.get_actual_velocities()) != 0 || pros::c::millis() <= t){}//Wait for motor to stop or 1 second to pass
		left.set_brake_modes(lbm);
		right.set_brake_modes(rbm);
	}
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		imu.reset(true);
		pros::lcd::set_text(2, "IMU Reset!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	imu.reset(true);
	left.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
	right.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
	left.tare_position();
	right.tare_position();
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello Nigel, Sage and Jaymaus!");

	pros::lcd::register_btn1_cb(on_center_button);
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
	/*while (true){
		std::cout << imu.get_pitch() << "\n";
		delay(1000);
	}*/
	if (imu.get_heading() != 0){
		imu.reset(true);
	}
	if (auton == "left"){
		head(335, 0.2);
		left.move(127);
		right.move(127);
		delay(750);
		while(left.get_actual_velocities()[0] > 100){
			//std::cout << left.get_actual_velocities()[0] <<"\n";
		}
		left.brake();
		right.brake();
		FWD(-10, 2);
		head(325);
		left.move(-127);
		right.move(-127);
		delay(500);
		while(left.get_actual_velocities()[0] > 0){
			//std::cout << left.get_actual_velocities()[0] <<"\n";
		}
		left.brake();
		right.brake();/*
		head(90);
		FWD(30);
		FWD(-10,100);*/
	}
	else if (auton == "right"){
		head(25, 0.4,true, 1000);
		left.move(127);
		right.move(127);
		delay(750);
		while(left.get_actual_velocities()[0] > 100){
			std::cout << left.get_actual_velocities()[0] <<"\n";
		}
		left.brake();
		right.brake();
		FWD(-5, 2);
		head(270,0,true, 1500);
		/*FWD(55, 1, false);
		head(240);*/
		left.move(127);
		right.move(127);
		while (imu.get_pitch() < 7.5){}
		delay(250);
		/*left.move(-127);
		right.move(127);
		delay(250);*/
		left.brake();
		right.brake();
		delay(500);
		left.move(-75);
		right.move(63.5);
		delay(250);
		left.brake();
		right.brake();
		//head(240);
		//FWD(30,0.5);
	}
	else{
		return;
	}
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		left.move(master.get_analog(ANALOG_LEFT_Y));
		right.move(master.get_analog(ANALOG_RIGHT_Y));

		if (H.get_digital(E_CONTROLLER_DIGITAL_R1)){
			PL.move(127);
			PR.move(127);
		}
		else{
			PL.move(0);
			PR.move(0);
		}
		

		pros::delay(20);
	}
}
