#include "main.h"
pros::Controller V(pros::E_CONTROLLER_MASTER);
pros::Controller H(pros::E_CONTROLLER_PARTNER);
pros::Motor RFF (11,true);
pros::Motor RFB (1);
pros::Motor RBF (15, true);
pros::Motor RBB (16, true);
pros::Motor LFF (20);
pros::Motor LFB (19, true);
pros::Motor LBF (10);
pros::Motor LBB (18);

pros::Motor_Group L ({LFF,LFB,LBF,LBB});
pros::Motor_Group R ({RFF,RFB,RBF,RBB});

float speed = 0.7;
bool Override = false;

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
	pros::lcd::set_text(0, "Push Bot Initialised!");
	pros::lcd::set_text(1, "Welcome to Hai Sing Catholic School!");

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

void controlLoop(){
	if (Override){
		L.move(V.get_analog(ANALOG_LEFT_Y)*speed);
		R.move(V.get_analog(ANALOG_RIGHT_Y)*speed);
	}
	else{
		L.move(H.get_analog(ANALOG_LEFT_Y)*speed);
		R.move(H.get_analog(ANALOG_RIGHT_Y)*speed);
	}
}

void UpdateSpeed(){
	//std::cout << speed << "\n";
	H.set_text(1,0,"Speed: "+std::to_string((int)std::ceil(speed*100))+"%  ");
	pros::delay(200);
	V.set_text(1,0,"Speed: "+std::to_string((int)std::ceil(speed*100))+"%  ");
}

void UpdateControl(){
	if (Override){
		H.set_text(2,0,"Control: Master");
		pros::delay(200);
		V.set_text(2,0,"Control: Master");
	}
	else{
		H.set_text(2,0,"Control: Slave ");
		pros::delay(200);
		V.set_text(2,0,"Control: Slave ");
	}
}

void opcontrol() {
	V.clear();
	pros::delay(100);
	H.clear();
	pros::delay(100);
	H.set_text(0,0,"Slave Controller");
	pros::delay(100);
	V.set_text(0,0,"Master Controller");
	pros::delay(100);
	UpdateSpeed();
	pros::delay(100);
	UpdateControl();
	while (true) {
		/*pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);*/
		controlLoop();

		if (V.get_digital(DIGITAL_R1)){
			Override = !Override;
			H.rumble("-");
			while((H.get_digital(DIGITAL_R1) || V.get_digital(DIGITAL_R1))){
				controlLoop();
			};
			UpdateControl();
			V.rumble("-");
		}

		if(H.get_digital(DIGITAL_UP) && speed < 0.6  && Override == false){
			speed += 0.1;
			UpdateSpeed();
			while(H.get_digital(DIGITAL_UP)){
				controlLoop();
			};
		}
		else if(V.get_digital(DIGITAL_UP) && speed < 1){
			speed += 0.1;
			UpdateSpeed();
			while(V.get_digital(DIGITAL_UP)){
				controlLoop();
			};
		}
		else if (((H.get_digital(DIGITAL_DOWN) && (Override == false)) || V.get_digital(DIGITAL_DOWN)) && speed > 0)
		{
			speed -= 0.1;
			UpdateSpeed();
			while((H.get_digital(DIGITAL_DOWN) || V.get_digital(DIGITAL_DOWN))){
				controlLoop();
			};
		}
		//UpdateSpeed();
	
		

		pros::delay(20);
	}
}
