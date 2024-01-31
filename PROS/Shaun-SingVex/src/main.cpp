#define _USE_MATH_DEFINES
#undef __STRICT_ANSI__
#include "main.h"
#include <numeric>
#include <math.h>
#include <map>

using namespace pros;

Motor L (3,true);
Motor L1 (1);
Motor L2 (2);
Motor_Group left ({L,L1,L2});
Motor R (13,false);
Motor R1 (11,true);
Motor R2 (12,true);
Motor_Group right ({R,R1,R2});

Motor FlyL (8, E_MOTOR_GEARSET_06);
Motor FlyR (9, E_MOTOR_GEARSET_06, true);

Rotation horizontal (19,false);
Imu imu (18);

ADIDigitalOut piston ('A');
ADIDigitalOut trackerCtrl ('B');
ADIDigitalOut hook ('C');

Vision vison (2);

Controller H (E_CONTROLLER_MASTER);

bool Pist = true;

/*---------------------------Start Auton Variables-------------------------*/
std::string autonSelectionStr = "Left";

double headin = 0;
//assume origin (0,0) is bottom right.
double offset_x = 0;
double offset_y = 0;
double x = offset_x;
double y = offset_y;
double wheelCircumference = 4.1*M_PI;
double wheelCircumference2 = 3.25*M_PI;
double wheelTrack = 10.4;
bool prevTurningIsClockwise = true;

std::map<double, double> keyPoints;

vision_signature_s_t REDOBJ = vison.signature_from_utility(1, 4869, 9157, 7013, -1301, -467, -884, 1.600, 0);
vision_signature_s_t BLUOBJ = vison.signature_from_utility(2, -3909, -3227, -3568, 8505, 10719, 9612, 3.000, 0);
vision_signature_s_t BLKOBJ = vison.signature_from_utility(3, -1, 431, 215, -51, 51, 0, 3.000, 0);
vision_signature_s_t GENTRI = vison.signature_from_utility(4, -7045, -5845, -6446, -8191, -7037, -7614, 3.000, 0);


/*----------------------------End Auton Variables--------------------------*/



/*---------------------------Start Auton Functions-------------------------*/

double avg(const std::vector<double>& v) {
    if (v.empty()) {
        return 0.0;
    }
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    return sum / v.size();
}

std::vector<double> off(const std::vector<double>& a,const std::vector<double>& b){
	return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};
}

double cdeg(double input){
	return (input/100)/360;
}

void reset(){
	left.tare_position();
	right.tare_position();
	horizontal.reset_position();
}
void reset2(){
	reset();
	headin = 0;
	x = offset_x;
	y = offset_y;
	lcd::set_text(7,"Resetting...");
	imu.reset(true);
	lcd::clear_line(7);
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

void GetLocation(bool print=true){
	double l = L.get_position();//(left.get_positions());
	double r = R.get_position();//(right.get_positions());
	double h = horizontal.get_position();
	//bool wasCalibrating = false;
	reset();
	while (imu.is_calibrating())
	{
		//pros::lcd::set_text(4,"IMU CALIBRATING...");
		//wasCalibrating = true;
	}
	headin = imu.get_heading();
	double head = headin*(M_PI/180);
	head += (((l*wheelCircumference)-(r*wheelCircumference))/wheelTrack);
	while (head > (360*(M_PI/180))){
		head -= (360*(M_PI/180));
	}
	while (head < 0){
		head += (360*(M_PI/180));
	}
	//headin = head*(180/M_PI);
	double x2 = cdeg(h)*wheelCircumference2;
	double y2 = ((l*wheelCircumference)+(r*wheelCircumference))/2;
	x += (x2*cos(head))-(y2*sin(head));
	y += (y2*cos(head))+(x2*sin(head));
	if (print == true){
		pros::lcd::set_text(2, "Heading: "+ std::to_string(headin)+"deg");
		pros::lcd::set_text(3, "Location: ("+ std::to_string(x) + ", " + std::to_string(y)+")");
		double targetHeadingFrom0 = atan2(0 - (x*-1), 10 - y)*(180/M_PI);
		double targetHeading = targetHeadingFrom0 + L_or_R(headin,targetHeadingFrom0);
		/*if (wasCalibrating){
			lcd::clear_line(7);
		}*/
		//lcd::set_text(6,"Heading: "+std::to_string(targetHeading));
	}
}


double distance(double x2, double y2){
	return sqrt(pow((x2-x),2)+pow((y2-y),2));
}


void goTo(double xIn, double yIn, bool exact=false){
	double l;
	double r;
	double targetHeadingFrom0 = atan2(xIn - x, yIn - y)*(180/M_PI);
	double targetHeading = L_or_R(headin,targetHeadingFrom0);
	double speed = 12;
	auto lbm = left.get_brake_modes()[0];//lbm = left brake mode
	auto rbm = right.get_brake_modes()[0];
	left.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	right.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	lcd::set_text(4,"Task: Pure Pursuit");
	lcd::set_text(5,"Target: ("+std::to_string(xIn)+", "+std::to_string(yIn)+")");
	lcd::set_text(6,"Target Heading: "+std::to_string(targetHeadingFrom0));
	
	while (distance(xIn,yIn) > 1){
		GetLocation(false);
		targetHeadingFrom0 = atan2(xIn - (x*-1), yIn - y)*(180/M_PI);
		targetHeading = L_or_R(headin,targetHeadingFrom0);
		//return;
		if (exact==false && ((targetHeading <= -80 || targetHeading >= 80))){
			// if heading is less than -90 or more than 90, target is behind actual,
			// may want to check if next target is in front. That is contolled by
			// exact variable.
			break;
		}
		else if (targetHeading <= -180 || targetHeading >= 180){
			if (prevTurningIsClockwise == true){
				l = speed;
				r = speed*-1;
			}
			else{//prevTurningIsClockwise == false
				l = speed*-1;
				r = speed;
			}
		}
		else if (targetHeading > 0){
			l = speed;
			r = ((1-(abs(targetHeading)/180))*(speed+12))-12;
		}
		else if (targetHeading < 0){
			l = ((1-(abs(targetHeading)/180))*(speed+12))-12;
			r = speed;
		}
		else{//targetHeading == 0
			l = speed;
			r = speed;
		}
		left.move_voltage(l*1000);
		right.move_voltage(r*1000);
		pros::delay(20);
	}
	if (true){
		left.brake();
		right.brake();
		double t = pros::c::millis() + 1000;
		while (left.get_actual_velocities()[0] != 0 || right.get_actual_velocities()[0] != 0 || pros::c::millis() <= t){}//Wait for motor to stop or 1 second to pass
		left.set_brake_modes(lbm);
		right.set_brake_modes(rbm);
	}
	lcd::clear_line(4);
	lcd::clear_line(5);
	return;
}


/*void head(double head, int timeout = 5000){
	auto lbm = left.get_brake_modes()[0];//lbm = left brake mode
	auto rbm = right.get_brake_modes()[0];
	left.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	right.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	double rotate = L_or_R(headin,head);
	double initial = rotate;
	double maxspeed = 12;
	if (rotate < 55 &&  rotate > -55){
		maxspeed = 3;
	}
	double minspeed = 1;
	double speed;
	int32_t l;
	int32_t r;
	lcd::set_text(4,"Task: Rotating");
	lcd::set_text(5,"Target: "+std::to_string(head)+"deg");
	double t = pros::c::millis() + timeout;
	while (pros::c::millis() <= t){
		GetLocation(false);
		rotate = L_or_R(headin,head);
		speed = ((rotate/initial)*maxspeed)+minspeed;
		if (left.get_actual_velocities()[0] == 0 && right.get_actual_velocities()[0] == 0 && (rotate/initial) > 0.5 && minspeed < 2){
			minspeed += 0.5;
		}
		if (rotate >= 1){
			l = speed;
			r = speed*-1;
		}
		else if (rotate <= -1){
			l = speed*-1;
			r = speed;
		}
		else{
			left.brake();
			right.brake();
			break;
		}
		left.move_voltage(l*1000);
		right.move_voltage(r*1000);
		//pros::delay(100);
	}
	lcd::clear_line(4);
	lcd::clear_line(5);
	t = pros::c::millis() + 1000;
	while (left.get_actual_velocities()[0] != 0 || right.get_actual_velocities()[0] != 0 || pros::c::millis() <= t){}//Wait for motor to stop or 1 second to pass
	left.set_brake_modes(lbm);
	right.set_brake_modes(rbm);
}*/

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


void right90(){
	head(90);
	//head(headin-90);
}

void trace(){
	goTo(22,20);
	//return;
	head(180);
	//pros::delay(2000);
	goTo(0,0);
	head(0);
	goTo(0,70);
	//goTo(-90,150);
	return;
	double xT;
	double yT;
	std::map<double, double>::iterator it = keyPoints.begin();
	while (it != keyPoints.end())
	{
		xT = it->first;
		yT = it->second;
		lcd::clear_line(3);
		lcd::print(3, "Target: (%1f, %1f)", xT, yT);
		goTo(xT,yT,true);
		++it;
	}
}
void printVison(){
	int it[7]= {0,1,2,3,4,5,6};
	std::string names[4] = {"RED","BLU","BLK","GEN"};
	//const char* names[4] = {n[0].c_str(),n[1].c_str(),n[2].c_str(),n[3].c_str()};
	//std::iota(0,vison.get_object_count(),it);
	for (int i: it){
		vision_object_s_t a = vison.get_by_size(i);
		if (a.signature != 255){
			lcd::print(i+1,"%s: (%1i, %1i, %1i, %1i)",names[a.signature-1],a.left_coord,a.top_coord,a.width,a.height);
		}
		else{
			lcd::clear_line(i+1);
		}
	}
}

float getSize(vision_object_s_t obj){
	return (obj.width)*(obj.height);
}
class VisonObject{
	public:
		std::string name;
	    VisonObject(std::string nameIn){
			name = nameIn;
		};
		int objects = 0;
		int x = 0;
		int y = 0;
		int w = 0;
		int h = 0;
		void addObj(vision_object_s_t obj){
			objects++;
			if (objects > 1){
				if (obj.left_coord < x){
					x = obj.left_coord;
				}
				if (obj.top_coord < y){
					y = obj.top_coord;
				}
				if (obj.left_coord+obj.width > x+w){
					w = (obj.left_coord+obj.width)-x;
				}
				if (obj.top_coord+obj.height > y+h){
					h = (obj.top_coord+obj.height)-y;
				}
			}
			else{
				x = obj.left_coord;
				y = obj.top_coord;
				w = obj.width;
				h = obj.height;
			}
		}
		void print(int line){
			lcd::clear_line(line);
			if (objects > 0){
				lcd::print(line,"%s: %1i (%1i, %1i, %1i, %1i)", name, objects, x, y, w, h);
			}
			else{
				lcd::print(line,"%s: %1i", name, objects);
			}
		}
};


void printVison2(){
	VisonObject red ("RED");
	VisonObject blu ("BLU");
	VisonObject blk ("BLK");
	VisonObject gen ("GEN");
	VisonObject types[4] = {red,blu,blk,gen};
	for (int i=0; i<vison.get_object_count();i++){
		vision_object_s_t obj = vison.get_by_size(i);
		if (obj.signature <5){
			types[obj.signature-1].addObj(obj);
		}
	}
	for (int i=0; i < 4; i++){
		types[i].print(i+1);
	}
}

void autonSelection(){
	while (true){
		H.set_text(0,0,"Auton: "+autonSelectionStr+"   ");
		if(H.get_digital(E_CONTROLLER_DIGITAL_LEFT)){
			autonSelectionStr = "Left";
		}
		else if(H.get_digital(E_CONTROLLER_DIGITAL_RIGHT)){
			autonSelectionStr = "Right";
		}
		else if(H.get_digital(E_CONTROLLER_DIGITAL_UP)){
			autonSelectionStr = "noAuton";
		}
		else if(H.get_digital(E_CONTROLLER_DIGITAL_A)){
			H.set_text(0,0,"AutonSet: "+autonSelectionStr);
			break;
		}
	}
}

void FWD(double target, double const speedConst = 0, bool stop=true, int timeout = 3000){
	motor_brake_mode_e_t lbm;
	motor_brake_mode_e_t rbm;
	std::vector<double> LOriginal = left.get_positions();
	std::vector<double> ROriginal = right.get_positions();
	if (stop){
		motor_brake_mode_e_t lbm = left.get_brake_modes()[0];//lbm = left brake mode
		motor_brake_mode_e_t rbm = right.get_brake_modes()[0];
		left.set_brake_modes(E_MOTOR_BRAKE_HOLD);
		right.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	}
	//double dist = (((avg(off(left.get_positions(),LOriginal))) + avg(off(right.get_positions(),ROriginal)))/2)*3.25*M_PI;
	double dist = ((((L.get_position()-LOriginal[0])) + (R.get_position()-ROriginal[0]))/2)*3.25*M_PI;
	double err = target - dist;
	double speed;
	if (speedConst == 0){
		speed = (1-(dist/target))+0.4;// min speed is 20% (2.4 Volts)
	}
	else{
		speed = speedConst;
	}
	//double speed = (1-(dist/target))+0.2;// min speed is 20% (2.4 Volts)
	double t = pros::c::millis() + timeout;
	while (pros::c::millis() <= t){
		std::cout<< err <<"\n";
		//std::cout << "[" << left.get_positions()[0] << "  " << left.get_positions()[1] << "  " << left.get_positions()[2] << "  " << right.get_positions()[0] << "  " << right.get_positions()[1] << "  " << right.get_positions()[2] << "]\n";
		dist = ((((L.get_position()-LOriginal[0])) + (R.get_position()-ROriginal[0]))/2)*3.25*M_PI;
		err = target - dist;
		//speed = (1-(dist/target))+0.2;// min speed is 20% (2.4 Volts)
		if (speedConst == 0){
		speed = (1-(dist/target))+0.4;// min speed is 20% (2.4 Volts)
		}
		else{
			speed = speedConst;
		}
		/*left.move(127*speed);
		right.move(127*speed);*/
		if (err >= -0.5 && err <= 0.5){
			break;
		}
		if (err > 0){
			left.move(127*speed);
			right.move(127*speed);
		}
		else if (err < 0){
			left.move(-127*speed);
			right.move(-127*speed);
		}
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


/*----------------------------End Auton Functions--------------------------*/



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
		pros::lcd::set_text(2, "Tracker Down!");
		trackerCtrl.set_value(true);
	} else {
		pros::lcd::clear_line(2);
		trackerCtrl.set_value(false);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	left.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
	right.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
	left.tare_position();
	right.tare_position();
	horizontal.reset();
	imu.reset(true);
	pros::lcd::initialize();

	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::set_text(0, "Hello Shuan, Nadia & Azlan!");
	pros::lcd::set_text(1, "Auton: "+autonSelectionStr);
	H.set_text(0,0,"Auton: "+autonSelectionStr);
	//autonomous();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	L1.brake();
	L2.brake();
	R1.brake();
	R2.brake();
	/*if(H.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
		autonSelection();
	}*/
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
	//autonSelection();
	/*lcd::set_text(0,"Auton: "+autonSelectionStr+"   ");
	if(lcd::read_buttons()==100){
		autonSelectionStr = "Left";
	}
	else if(lcd::read_buttons()==001){
		autonSelectionStr = "Right";
	}*/
	/*else if(H.get_digital(E_CONTROLLER_DIGITAL_A)){
		H.set_text(0,0,"AutonSet: "+autonSelectionStr);
		break;
	}*/
}

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
	left.tare_position();
	right.tare_position();
	horizontal.reset();
	trackerCtrl.set_value(true);
	if (imu.get_heading() != 0){
		imu.reset(true);
	}
	/*while (true)
	{
		std::cout << imu.get_roll() <<"\n";
		delay(1000);
	}*/
    //FWD(10);
	//return;
	if (autonSelectionStr == "Left"){
		head(335, 0.4);
		left.move(127);
		right.move(127);
		delay(750);
		while(left.get_actual_velocities()[0] > 75){
			//std::cout << left.get_actual_velocities()[0] <<"\n";
		}
		left.brake();
		right.brake();
		FWD(-10, 2);
		/*head(325);
		left.move(-127);
		right.move(-127);
		delay(1500);
		while(left.get_actual_velocities()[0] > 0){
			//std::cout << left.get_actual_velocities()[0] <<"\n";
		}
		left.brake();
		right.brake();
		head(270);
		left.move(-127);
		right.move(-127);
		delay(500);*/
		/*while(left.get_actual_velocities()[0] > 100){
			std::cout << left.get_actual_velocities()[0] <<"\n";
		}*/
		/*left.brake();
		right.brake();*/
		//goTo(12,0);
		//FWD(10,true,1000);
		//FWD(-10,100);
		/*FWD(60, false);
		head(260);
		FWD(10, true, 1000);
		head(225);
		goTo(-24,12);
		head(130);
		hook.set_value(true);
		head(90);
		delay(500);
		head(0);
		hook.set_value(false);
		head(90);
		goTo(48,0);*/
		
	}
	else if (autonSelectionStr == "Right"){
		head(25, 0.4,true, 1000);
		left.move(127);
		right.move(127);
		double t = pros::c::millis() + 2500;
		delay(750);
		while(left.get_actual_velocities()[0] > 100){
			//std::cout << left.get_actual_velocities()[0] <<"\n";
			if (pros::c::millis() >= t){
				break;
			}
		}
		left.brake();
		right.brake();
		FWD(-5);
		head(270,0,true, 1500);
		//FWD(10, 1, false);
		left.move(127);
		right.move(127);
		FWD(65,1,false);
		/*while (imu.get_roll() > -15){}
		while (true){
			if (imu.get_roll() > -15 && imu.get_roll() < -10){
				break;
			}
		}*/
		//delay(250);
		/*left.move(-127);
		right.move(100);
		delay(750);*/
		head(220);
		double err = L_or_R(imu.get_heading(),220);
		if (err >= 12.5 && err <= -12.5){
			left.move(127);
			right.move(127);
			delay(5000);
		}
		else{
			left.brake();
			right.brake();
			//double t = pros::c::millis() + 5000;
		}
		//delay(2000);
		/*while(left.get_actual_velocities()[0] > 100){
			//std::cout << left.get_actual_velocities()[0] <<"\n";
			if (pros::c::millis() >= t){
				break;
			}
		}*/
		/*left.brake();
		right.brake();
		FWD(7,0.5);
		head(229);
		FWD(5,1);*/
		/*FWD(60, false);
		head(100);
		FWD(10, true, 1000);
		head(135);
		goTo(24,12);
		hook.set_value(true);
		head(0);
		hook.set_value(false);
		head(225);
		goTo(-40,0);
		head(180);
		hook.set_value(true);*/

	}
	else{
		return;
	}
	trackerCtrl.set_value(false);
}


int getMotorTemp(){
	return (L.get_temperature()+L1.get_temperature()+L2.get_temperature()+R.get_temperature()+R1.get_temperature()+R2.get_temperature()+FlyL.get_temperature()+FlyR.get_temperature())/8;
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
	trackerCtrl.set_value(false);
	hook.set_value(false);
	H.clear();
	pros::delay(200);
	H.set_text(1,0,"CTL: TANK  ");
	pros::delay(200);
	double target;
	double l;
	double r;
	double x;
	double y;
	double speed;
	while (true) {
		if (Pist){
			L1.move(H.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
			L2.move(H.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
			R1.move(H.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
			R2.move(H.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
			L.move(H.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
			R.move(H.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
		}
		else{
			/*if (E_CONTROLLER_ANALOG_RIGHT_Y != 0){
				L1.move(H.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)*-1);
				L2.move(H.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)*-1);
				R1.move(H.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)*-1);
				R2.move(H.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)*-1);
			}
			else{
				L1.brake();
				L2.brake();
				R1.brake();
				R2.brake();
			}*/
			x = H.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
			y = H.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
			speed = sqrt(pow(x,2)+pow(y,2));
			if (y !=0 || x != 0){
				target = (std::atan2(x,y))*(180/M_PI);
				if (target < 0) target += 360;
				//if (target > 180) target -= 360;
				if (target >= 0 && target < 90){
					l = speed;
					r = ((speed*2)*(1-(target/90)))-y;
				}
				else if (target >= 90 && target < 180){
					l = ((speed*2)*(1-((target-90)/90)))-speed;
					r = -(speed);
				}
				else if (target >= 180 && target < 270){
					l = -(speed);
					r = ((speed*2)*(((target-180)/90)))-speed;
				}
				else if (target >= 270 && target < 360){
					l = ((speed*2)*(((target-270)/90)))-speed;
					r = speed;
				}
				else{//should not happen, but just in case.
					l = 0;
					r = 0;
				}
			}
			else {
				l = 0;
				r = 0;
			}
			left.move(l);
			right.move(r);
			//lcd::set_text(0,"ATAN2: "+std::to_string(target));
			//lcd::set_text(1,"L: "+std::to_string(l));
			//lcd::set_text(2,"R: "+std::to_string(r));

		}
		if (H.get_digital(DIGITAL_L1)){
			if (Pist){
				//piston.set_value(true);
				Pist = false;
				H.set_text(1,0,"CTL: ARCADE");
				/*L1.set_brake_mode(E_MOTOR_BRAKE_HOLD);
				L2.set_brake_mode(E_MOTOR_BRAKE_HOLD);
				R1.set_brake_mode(E_MOTOR_BRAKE_HOLD);
				R2.set_brake_mode(E_MOTOR_BRAKE_HOLD);*/
			}
			else {
				//piston.set_value(false);
				Pist = true;
				H.set_text(1,0,"CTL: TANK  ");
				L1.set_brake_mode(E_MOTOR_BRAKE_COAST);
				L2.set_brake_mode(E_MOTOR_BRAKE_COAST);
				R1.set_brake_mode(E_MOTOR_BRAKE_COAST);
				R2.set_brake_mode(E_MOTOR_BRAKE_COAST);
			}
			while (H.get_digital(DIGITAL_L1)){}
		}
		if (H.get_digital(DIGITAL_R1)){
			FlyL.move_voltage(12000);
			FlyR.move_voltage(12000);
		}
		else{
			FlyL.move_voltage(0);
			FlyR.move_voltage(0);
		}
		if (H.get_digital(E_CONTROLLER_DIGITAL_L2)){
			piston.set_value(true);
		}
		else{
			piston.set_value(false);
		}
		H.set_text(2,0,"Battery: "+std::to_string((int)std::ceil(pros::battery::get_capacity()))+"%  ");
		lcd::set_text(3,"Avg Motor Temp: "+std::to_string((int)std::ceil(getMotorTemp()))+"deg  ");

		pros::delay(20);
	}
}
