#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

/****ACTUAL ROBOT CODE****/
//defining global variables
int sortedColor; //0 = keep blue fling red, 1 = keep red fling blue, 2 = keep all no fling
bool autonColor; //T = blue, F = red
bool autonSide; //T = close, F = far

void initialize() {
	pros::lcd::initialize();
	pros::lcd::print(5, "Initialized");
}

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

void donut_detected() { // defines
	pros::Motor chain (14, pros::MotorGearset::green); //5.5
	pros::Controller ctrl (CONTROLLER_MASTER);

	chain.set_brake_mode(pros::MotorBrake::brake);
	ctrl.rumble(".");
	pros::delay(90);
	chain.brake();
	pros::delay(1);
}

void donut_not_detected() { //define
	pros::Motor chain (14, pros::MotorGearset::green); //5.5
	//chain code (roller is indep. of color sort)
	chain.set_brake_mode(pros::MotorBrake::coast);	
}
void autonomous() {
	//comp control mode flag
	pros::lcd::print(5, "Driver Control");

	//insert auton here
}

void opcontrol() {
	//comp control mode flag
	pros::lcd::print(5, "Driver Control");

	//defining motors
	pros::MotorGroup left ({1, 2, 3}, pros::MotorGearset::blue);
	pros::MotorGroup right({11, 12, 13}, pros::MotorGearset::blue);
	pros::Motor roller (4, pros::MotorGearset::green); //5.5
	pros::Motor chain (14, pros::MotorGearset::green); //5.5
	pros::Motor lb (6, pros::MotorGearset::green); //5.5
	pros::Motor mogo (5, pros::MotorGearset::green); //5.5

	//defining sensors (add rot)
	pros::Vision vision (10);
	pros::Rotation lbRot (9);

	//defining controller
	pros::Controller ctrl (CONTROLLER_MASTER);

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

		//mogo mech (future)
		if(ctrl.get_digital(DIGITAL_Y)) {
			mogo.move_absolute(225, 200);
		}
		else{
			mogo.move_absolute(0, 200);
		}

		pros::delay(2);
	}
}