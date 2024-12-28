#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include <cmath>
#include <numbers>

/****ACTUAL ROBOT CODE****/
//defining global variables
int sortedColor; //0 = keep blue fling red, 1 = keep red fling blue, 2 = keep all no fling
bool autonColor; //T = blue, F = red
bool autonSide; //T = close, F = far
const int wheelCirc = 220; // in mm
const int driveEncoders = 300; //ticks per rev
const double trackWidth = 9.81 * 25.4; //conversion to mm
const int lbNumStates = 3;
int lbStates[lbNumStates] = {0, 3000, 20000}; //in centidegrees (remove 2 zeroes for deg value)
int lbCurrState = 0;
int lbTarget = 0;

//defining constructors for everything lol
//defining motors
pros::MotorGroup left ({1, 2, 3}, pros::MotorGearset::blue);
pros::MotorGroup right({11, 12, 13}, pros::MotorGearset::blue);
pros::Motor roller (4, pros::MotorGearset::green); //5.5
pros::Motor chain (14, pros::MotorGearset::green); //5.5
pros::Motor lb (6, pros::MotorGearset::green); //5.5
pros::Motor mogo (5, pros::MotorGearset::green); //5.5
//defining sensors
pros::Vision vision (10);
pros::Rotation lbRot (9);
//defining controller
pros::Controller ctrl (CONTROLLER_MASTER);

//lcd stuffs
void on_left_button() {
	if(sortedColor >= 2 && sortedColor <= 0) {
		sortedColor++;
	}
	else if(sortedColor < 2) {
		sortedColor = 0;
	}
	else {
		sortedColor = 0;
	}

	if(sortedColor == 0) {
		pros::lcd::print(1, "Left Button: Fling Red, Keep Blue");
	}
	else if(sortedColor == 1) {
		pros::lcd::print(1, "Left Button: Fling Blue, Keep Red");
	}
	else {
		pros::lcd::print(1, "Left button: No Color Sort");
	}
}

void on_center_button() {
	autonColor = !autonColor;
	if(autonColor) {
		pros::lcd::print(2, "Center Button: Blue Side Auton");
	}
	else {
		pros::lcd::print(2, "Center Button: Red Side Auton");
	}
}

void on_right_button() {
	autonSide = !autonSide;
	if(autonSide) {
		pros::lcd::print(3, "Right Button: Close Side Auton");
	}
	else {
		pros::lcd::print(3, "Right Button: Far Side Auton");
	}
}

//driver control funcs
void donut_detected() { // define
	chain.set_brake_mode(pros::MotorBrake::brake); //to effectively fling
	ctrl.rumble("."); //alerts driver
	pros::delay(90); //adjustable
	chain.brake();
	pros::delay(1);
}

void donut_not_detected() { //define
	//resets it to coast when not sorting
	chain.set_brake_mode(pros::MotorBrake::coast);	
}

void lbNextState(bool dir) {
	if(dir) {
		if(lbCurrState >= (lbNumStates - 1)) {lbCurrState = 0;}
		else {lbCurrState++;}
	}
	else {
		if(lbCurrState <= 0) {lbCurrState = (lbNumStates - 1);}
		else {lbCurrState--;}
	}
	lbTarget = lbStates[lbCurrState];
}

void lbControl() {
	double kp = 1.4; //reactivity of the control
	double error = lbTarget - lbRot.get_position();
	double velocity = kp * error;
	lb.move(velocity);
}

//autonomous functions
void drive(int inDist, bool dir, int rpm) {
	left.tare_position();
	right.tare_position();
	double mmDist = inDist * 25.4;
	double rotations = round(10*(mmDist / wheelCirc)) * 0.1;
	double ticks = round(rotations * driveEncoders);
	double pause = (rotations / rpm) * 60000;

	if(dir) { //front
		left.move_absolute(ticks, rpm);
		right.move_absolute(ticks, rpm);
	}
	else { //back
		left.move_absolute(-1 * ticks, rpm);
		right.move_absolute(-1 * ticks, rpm);
	}

	pros::delay(pause);
}

void turn(double degrees, bool dir, int rpm) {
	left.tare_position();
	right.tare_position();
	double mmWheelCirc = wheelCirc * 25.4;
	double turnCirc = std::numbers::pi * trackWidth;
	double arcLen = (degrees/360) * turnCirc;
	double rotations = arcLen / mmWheelCirc;
	double ticks = round(rotations * driveEncoders);
	double pause = (rotations / rpm) * 60000;

	if(dir) { //left
		left.move_absolute(-1 * ticks, rpm);
		right.move_absolute(ticks, rpm);
	}
	else { //right
		left.move_absolute(ticks, rpm);
		right.move_absolute(-1 * ticks, rpm);
	}

	pros::delay(pause);
}

//actual code
void initialize() {
	pros::lcd::initialize();
	pros::lcd::print(5, "Initialized");
	ctrl.rumble("-");

	pros::Task lbControlTask ([]{
		while(true) {
			lbControl();
			pros::delay(10);
		}
	});
}

void autonomous() {
	//comp control mode flag
	pros::lcd::print(5, "Autonomous");

	//test auto
	drive(24, true, 200);
	turn(90.0, true, 100);
	drive(24, false, 200);
	turn(90.0, true, 100);
	drive(24, true, 200);
	turn(90.0, false, 100);
	drive(24, true, 200);
	turn(720.0, false, 100);
}

void opcontrol() {
	//comp control mode flag
	pros::lcd::print(5, "Driver Control");

	//setting motor brake (chain changes so it is set in whiletrue)
	left.set_brake_mode_all(pros::MotorBrake::coast);
	right.set_brake_mode_all(pros::MotorBrake::coast);
	roller.set_brake_mode(pros::MotorBrake::coast);
	lb.set_brake_mode(pros::MotorBrake::brake);
	mogo.set_brake_mode(pros::MotorBrake::brake);
	mogo.set_zero_position(0);

	while(true) {
		/*
		lcd layout (max 8 lines):
		0: hi (can be changed/removed later)
		1: left button setting - color sort fling
		2: mid button setting - auton color
		3: right button setting - auton side
		4: temp flags - overheat or not
		5: comp ctrl mode flag - what mode it is in right now
		*/
		pros::lcd::print(0, "hi");
		
		//temp flags
		float dtLeftOT = ((round(10.0*((left.is_over_temp(0) + left.is_over_temp(1) + left.is_over_temp(2))/3.0)))/10.0);
		float dtRightOT = ((round(10.0*((right.is_over_temp(0) + right.is_over_temp(1) + right.is_over_temp(2))/3.0)))/10.0);
		int rollerOT = roller.is_over_temp();
		int chainOT = chain.is_over_temp();
		int lbOT = lb.is_over_temp();
		int mogoOT = mogo.is_over_temp();
		//printing the overtemp flags on to lcd
		pros::lcd::print(4, "DTL%d DTR%d Roller%d Chain%d LB%d Mogo%d", dtLeftOT, dtRightOT, rollerOT, chainOT, lbOT, mogoOT);

		//drivetrain numbers
		int power = ctrl.get_analog(ANALOG_RIGHT_Y);
		int turn;
		if(ctrl.get_digital(DIGITAL_RIGHT)) {
			turn = (ctrl.get_analog(ANALOG_LEFT_X)) / 2;
		}
		else {
			turn = ctrl.get_analog(ANALOG_LEFT_X);
		}
		int powerL = power + turn;
		int powerR = power - turn;
		
		//dt
		left.move(powerL);
		right.move(powerR);

		//color sort
		pros::vision_object_s_t flingBlue = vision.get_by_sig(0, 1); //sorts out red donuts
		pros::vision_object_s_t flingRed = vision.get_by_sig(0, 2); //sorts out blue donuts
		if (chain.get_brake_mode() == pros::MotorBrake::coast){
			if(ctrl.get_digital(DIGITAL_R2)) {
				chain.move(200);
			}
			else if(ctrl.get_digital(DIGITAL_R1)) {
				chain.move(-200);
			}
		}
		//disable color sort double keybind
		if(ctrl.get_digital(DIGITAL_X) && ctrl.get_digital(DIGITAL_UP)) {
			sortedColor = 2;
		}
		else {
			sortedColor = sortedColor;
		}
		pros::Task color_sort([flingRed, flingBlue]{ // color checking task
			if(sortedColor) { //keep blue fling red
				if(flingRed.signature == 1) {donut_detected();}
				else {donut_not_detected();}
			}
			else { //the other 2
				if(sortedColor == 1) {
					if(flingBlue.signature == 1) {donut_detected();}
					else {donut_not_detected();}
				}
				else if(sortedColor == 2) {donut_not_detected();}
			}
			pros::delay(10);
		});

		//roller
		if(ctrl.get_digital(DIGITAL_R2)) {
			roller.move(200);
		}
		else if(ctrl.get_digital(DIGITAL_R1)) {
			roller.move(-200);
		}

		//lady brown (future)
		bool toggle = true;
		if(ctrl.get_digital_new_press(DIGITAL_DOWN)) {toggle = !toggle;}

		if(toggle) {
			//using set positions
			if(ctrl.get_digital_new_press(DIGITAL_L1)) {lbNextState(false);} //down cycling
			else if(ctrl.get_digital_new_press(DIGITAL_L2)) {lbNextState(true);} //up cycling
		}
		else {
			if(ctrl.get_digital(DIGITAL_L1)) {lb.move_velocity(-100);} //down controlled motion
			else if(ctrl.get_digital(DIGITAL_L2)) {lb.move_velocity(100);} //up controlled motion
			else {lb.move(0);} //stationary
		}

		//mogo mech (future)
		if(ctrl.get_digital(DIGITAL_Y)) {
			mogo.move_absolute(225, 200); //rotates 1/4 a rotation
		}
		else{
			mogo.move_absolute(0, 200);
		}

		pros::delay(2);
	}
}