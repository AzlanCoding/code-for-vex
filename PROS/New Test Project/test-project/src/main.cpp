#define _USE_MATH_DEFINES
#undef __STRICT_ANSI__
#include "main.h"
#include <numeric>
#include <math.h>
#include <map>
using namespace pros;

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


Motor left (1);
Motor right (20,true);
Rotation horizontal (11,false);
Imu imu (10);
Vision vison (2);

vision_signature_s_t REDOBJ = vison.signature_from_utility(1, 4869, 9157, 7013, -1301, -467, -884, 1.600, 0);
vision_signature_s_t BLUOBJ = vison.signature_from_utility(2, -3909, -3227, -3568, 8505, 10719, 9612, 3.000, 0);
vision_signature_s_t BLKOBJ = vison.signature_from_utility(3, -1, 431, 215, -51, 51, 0, 3.000, 0);
vision_signature_s_t GENTRI = vison.signature_from_utility(4, -7045, -5845, -6446, -8191, -7037, -7614, 3.000, 0);


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
	double l = left.get_position();
	double r = right.get_position();
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
	auto lbm = left.get_brake_mode();//lbm = left brake mode
	auto rbm = right.get_brake_mode();
	left.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right.set_brake_mode(E_MOTOR_BRAKE_HOLD);
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
		while (left.get_actual_velocity() != 0 || right.get_actual_velocity() != 0 || pros::c::millis() <= t){}//Wait for motor to stop or 1 second to pass
		left.set_brake_mode(lbm);
		right.set_brake_mode(rbm);
	}
	lcd::clear_line(4);
	lcd::clear_line(5);
	return;
}


void head(double head){
	auto lbm = left.get_brake_mode();//lbm = left brake mode
	auto rbm = right.get_brake_mode();
	left.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right.set_brake_mode(E_MOTOR_BRAKE_HOLD);
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
	while (true){
		GetLocation(false);
		rotate = L_or_R(headin,head);
		speed = ((rotate/initial)*maxspeed)+minspeed;
		if (left.get_actual_velocity() == 0 && right.get_actual_velocity() == 0 && (rotate/initial) > 0.5 && minspeed < 2){
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
	double t = pros::c::millis() + 1000;
	while (left.get_actual_velocity() != 0 || right.get_actual_velocity() != 0 || pros::c::millis() <= t){}//Wait for motor to stop or 1 second to pass
	left.set_brake_mode(lbm);
	right.set_brake_mode(rbm);
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
	vison.set_signature(1,&REDOBJ);
	vison.set_signature(2,&BLUOBJ);
	vison.set_signature(3,&BLKOBJ);
	vison.set_signature(4,&GENTRI);
	vison.set_wifi_mode(1);
	pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(right90);
	pros::lcd::register_btn0_cb(trace);
	pros::lcd::register_btn2_cb(reset2);
	left.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);
	right.set_encoder_units(E_MOTOR_ENCODER_ROTATIONS);

	keyPoints.insert({0, 5});
	/*keyPoints.insert({1, 10});
	keyPoints.insert({1, 15});
	keyPoints.insert({1, 20});
	keyPoints.insert({1, 25});
	keyPoints.insert({1, 30});*/
	reset();
	while (imu.is_calibrating())
	{
		/* wait for imu to start */
	}
	
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
	/*pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(10,true);*/
	//right90();

	while (true) {
		/*pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);*/
		//int left = master.get_analog(ANALOG_LEFT_Y);
		//int right = master.get_analog(ANALOG_RIGHT_Y);

		/*left_mtr = master.get_analog(ANALOG_LEFT_Y);
		right_mtr = master.get_analog(ANALOG_RIGHT_Y);*/
		pros::lcd::set_text(0,"OBJ: "+std::to_string(vison.get_object_count()));
		GetLocation(false);
		printVison2();


		pros::delay(500);
	}
}
