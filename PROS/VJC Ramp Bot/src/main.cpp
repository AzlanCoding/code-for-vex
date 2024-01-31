#include "main.h"

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
		pros::lcd::set_text(2, "I was pressed!");
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
	pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");

	//pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
	pros::Motor LF(17);
	pros::Motor LB(18);
	pros::Motor RF(19,true);
	pros::Motor RB(20,true);
	pros::Motor_Group L({LF,LB});
	pros::Motor_Group R({RF,RB});

	pros::Motor ArmL(9);
	pros::Motor ArmR(10,true);

	pros::Motor motors[4] = {LF,RF,RB,LB};
	//master.clear();
	master.set_text(1,1,"Brake: OFF");
	//master.clear();
	//master.print(1,1,"BRAKE: OFF");
	bool braking;

	while (true) {
		//pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		//int left = master.get_analog(ANALOG_LEFT_Y);
		//int right = master.get_analog(ANALOG_RIGHT_Y);

		//left_mtr = left;
		//right_mtr = right;
		if (master.get_digital(DIGITAL_B)){
			if (braking == false){
			L.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
			R.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
			L.brake();
			R.brake();
			//master.clear_line(1);
			//master.set_text(1,1,"Brake: ON");
			//master.clear();
			master.print(1,1,"BRAKE: ON ");
			braking = true;
			//H.rumble(rumbleLong);
			//while(H.ButtonB.pressing()){}
			}
			else{
			L.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			R.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			L.brake();
			R.brake();
			//master.clear();
			master.print(1,1,"BRAKE: OFF");
			//master.set_text(1,1,"Brake: OFF");
			braking = false;
			//H.rumble(rumbleLong);
			}
			while(master.get_digital(DIGITAL_B)){}
		}
		else if (braking == false){
			L.move(master.get_analog(ANALOG_LEFT_Y));
			//L.move(master.get_analog(ANALOG_LEFT_Y));
			R.move(master.get_analog(ANALOG_RIGHT_Y));
			//R.move(master.get_analog(ANALOG_RIGHT_Y));
		}
		if (master.get_digital(DIGITAL_L1)){
			ArmL.move(127);
			ArmR.move(0);
		}
		else if (master.get_digital(DIGITAL_L2)){
			ArmL.move(-127);
			ArmR.move(0);
		}
		else if (master.get_digital(DIGITAL_R1)){
			ArmL.move(0);
			ArmR.move(127);
		}
		else if (master.get_digital(DIGITAL_R2)){
			ArmL.move(0);
			ArmR.move(-127);
		}
		else{
			ArmL.move(0);
			ArmR.move(0);
		}

		pros::delay(20);
	}
}