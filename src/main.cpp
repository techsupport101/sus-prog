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
int sortedColor = 0; //0 = keep blue fling red, 1 = keep red fling blue, 2 = keep all no fling
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
pros::MotorGroup left ({-1, 2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right({8, -9, 10}, pros::MotorGearset::blue);
pros::Motor chain (7, pros::MotorGearset::blue);
pros::Motor lb (12, pros::MotorGearset::green); //5.5
pros::Motor mogo (6, pros::MotorGearset::green); //5.5
//defining sensors
pros::Vision vision (4);
pros::Rotation lbRot (11);
//defining controller
pros::Controller ctrl (CONTROLLER_MASTER);

//lcd stuffs
void on_left_button() {
    if(sortedColor <= 2 && sortedColor >= 0) {
        sortedColor++;
    }
    else {
        sortedColor = 0;
    }
}

void on_center_button() {
    autonColor = !autonColor;
}

void on_right_button() {
    autonSide = !autonSide;
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
		lbCurrState++;
        if(lbCurrState >= (lbNumStates)) {lbCurrState = 0;}
    }
    else {
        lbCurrState--;
		if(lbCurrState < 0) {lbCurrState = (lbNumStates - 1);}
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
//to others; vs code adds the comment above a func to the func's tooltip when called upon

//Moves the robot forward and backward
void drive(int inDist, bool forward, int rpm) {
    left.tare_position();// same logic here when naming the bool as "forward"
    right.tare_position();
    double mmDist = inDist * 25.4;
    double rotations = round(10*(mmDist / wheelCirc)) * 0.1;
    double ticks = round(rotations * driveEncoders);
    double pause = (rotations / rpm) * 60000;

    if(forward) { //front
        left.move_absolute(ticks, rpm);
        right.move_absolute(ticks, rpm);
    }
    else { //back
        left.move_absolute(-1 * ticks, rpm);
        right.move_absolute(-1 * ticks, rpm);
    }

    pros::delay(pause + 1000);
}

//Rotates the robot left or right
void turn(double degrees, bool turnLeft, int rpm) {
    left.tare_position();
    right.tare_position();
    double turnCirc = std::numbers::pi * trackWidth;
    double arcLen = (degrees/360) * turnCirc;
    double rotations = arcLen / wheelCirc;
    double ticks = round(rotations * driveEncoders);
    double pause = (rotations / rpm) * 60 * 1000;

    if(turnLeft) { //left
        left.move_absolute(-1 * ticks, rpm);
        right.move_absolute(ticks, rpm);
    }
    else { //right
        left.move_absolute(ticks, rpm);
        right.move_absolute(-1 * ticks, rpm);
    }

    pros::delay(pause + 100);
}

//actual code
void initialize() {
    pros::lcd::initialize();
    pros::lcd::print(5, "Initialized");

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
    drive(24, true, 100);
    turn(90, false, 50);
}

void opcontrol() {
    //comp control mode flag
    pros::lcd::print(5, "Driver Control");

    //setting motor brake (chain changes so it is set in whiletrue)
    left.set_brake_mode_all(pros::MotorBrake::coast);
    right.set_brake_mode_all(pros::MotorBrake::coast);
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

        //button output stuffs
        if(sortedColor == 0) {
            pros::lcd::print(1, "Left Button: Fling Red, Keep Blue");
        }
        else if(sortedColor == 1) {
            pros::lcd::print(1, "Left Button: Fling Blue, Keep Red");
        }
        else {
            pros::lcd::print(1, "Left button: No Color Sort");
        }

        if(autonColor) {
            pros::lcd::print(2, "Center Button: Blue Side Auton");
        }
        else {
            pros::lcd::print(2, "Center Button: Red Side Auton");
        }

        if(autonSide) {
            pros::lcd::print(3, "Right Button: Close Side Auton");
        }
        else {
            pros::lcd::print(3, "Right Button: Far Side Auton");
        }
        
        //temp flags
        float dtLeftOT = ((round(10.0*((left.get_temperature(0) + left.get_temperature(1) + left.get_temperature(2))/3.0)))/10.0);
        float dtRightOT = ((round(10.0*((right.get_temperature(0) + right.get_temperature(1) + right.get_temperature(2))/3.0)))/10.0);
        float chainOT = chain.get_temperature();
        float lbOT = lb.get_temperature();
        float mogoOT = mogo.get_temperature();
        //printing the overtemp flags on to lcd
        pros::lcd::print(4, "DTL%.1f DTR%.1f Chain%.1f LB%.1f Mogo%.1f", dtLeftOT, dtRightOT, chainOT, lbOT, mogoOT);

        //drivetrain numbers
        int power = ctrl.get_analog(ANALOG_LEFT_Y);
        int turn;
        if(ctrl.get_digital(DIGITAL_Y)) {
            turn = (ctrl.get_analog(ANALOG_RIGHT_X)) / 2;
        }
        else {
            turn = ctrl.get_analog(ANALOG_RIGHT_X);
        }
        int powerL = power + turn;
        int powerR = power - turn;
        
        //dt
        left.move(powerL);
        right.move(powerR);

        //color sort
        pros::vision_object_s_t flingBlue = vision.get_by_sig(0, 1); //sorts out red donuts
        pros::vision_object_s_t flingRed = vision.get_by_sig(0, 2); //sorts out blue donuts

        //intake
        if (chain.get_brake_mode() == pros::MotorBrake::coast){
            if(ctrl.get_digital(DIGITAL_L1)) {
                chain.move_velocity(600);
            }
            else if(ctrl.get_digital(DIGITAL_L2)) {
                chain.move_velocity(-600);
            }
            else {
                chain.brake();
            }
        }
        //disable color sort double keybind
        if(ctrl.get_digital(DIGITAL_X) && ctrl.get_digital(DIGITAL_UP)) {
            sortedColor = 2;
        }
        else {
            sortedColor = sortedColor;
        }

        //actual color sort
        pros::Task color_sort([flingRed, flingBlue]{ // color checking task
            if(sortedColor == 0) { //keep blue fling red
                if(flingRed.signature == 1) {donut_detected();}
                else {donut_not_detected();}
            }
            else { //the other 2
                if(sortedColor == 1) {
                    if(flingBlue.signature == 1) {donut_detected();}
                    else {donut_not_detected();}
                }
                else {donut_not_detected();} // catch all just in case            
			}
            pros::delay(10);
        });

        //lady brown (future)
        bool toggle = true;
        if(ctrl.get_digital_new_press(DIGITAL_DOWN)) {toggle = !toggle;}

        if(toggle) {
            //using set positions
            if(ctrl.get_digital_new_press(DIGITAL_R1)) {lbNextState(false);} //down cycling
            else if(ctrl.get_digital_new_press(DIGITAL_R2)) {lbNextState(true);} //up cycling
        }
        else {
            if(ctrl.get_digital(DIGITAL_R1)) {lb.move_velocity(-100);} //down controlled motion
            else if(ctrl.get_digital(DIGITAL_R2)) {lb.move_velocity(100);} //up controlled motion
            else {lb.move(0);} //stationary
        }

        //mogo mech
        mogo.set_zero_position(mogo.get_position());
        if(ctrl.get_digital(DIGITAL_RIGHT)) {mogo.move_absolute(-900, 200);}
        else {mogo.move_absolute(0, 200);}

        pros::delay(2);
    }
}