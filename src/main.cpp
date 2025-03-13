#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/optical.hpp"
#include <cstdio>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-9, 8, -10},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({15, -16, 19}, 
                            pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Motor intake(-11, pros::MotorGearset::blue);

pros::Motor lift1(12, pros::MotorGearset::blue);
pros::Motor lift2(2, pros::MotorGearset::blue);

// Inertial Sensor on port 10
pros::Imu imu(5);

// Optical Sensor on port #
pros::Optical detector(6);

pros::ADIDigitalOut clamp('A');
pros::ADIDigitalOut leftD('C');
pros::ADIDigitalOut rightD('G');
pros::ADIDigitalOut sillyString('B');
pros::ADIButton selector('H');



// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(18);
// vertical tracking wheel encoder
//pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -3.25);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, -5.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             20, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);
// 4 - 26
// 3 - 20
// 2 - 20

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(0, // joystick deadband out of 127
                                     5, // minimum output where drivetrain will move out of 127
                                     1.02 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127
                                  5, // minimum output where drivetrain will move out of 127
                                  1.02 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Variable to store the clamp state
bool clampState = false;
bool leftState = false;
bool rightState = false;
bool sillyState = false;

void toggleClamp() {
    clampState = !clampState;  // Toggle state
    clamp.set_value(clampState);
    pros::delay(10);
}

void toggleLeft() {
    leftState = !leftState;  // Toggle state
    leftD.set_value(leftState);
}

void toggleRight() {
    rightState = !rightState;  // Toggle state
    rightD.set_value(rightState);
}

void toggleSilly() {
    sillyState = !sillyState;
    sillyString.set_value(sillyState);
}

void belt(double speed) {
    lift1.move(speed);
    lift2.move(speed);
}

/*
  ██████╗   ██████╗   ███████╗       █████╗   ██╗   ██╗  ████████╗   ██████╗   ███╗   ██╗
  ██╔══██╗  ██╔══██╗  ██╔════╝      ██╔══██╗  ██║   ██║  ╚══██╔══╝  ██╔═══██╗  ████╗  ██║
  ██████╔╝  ██████╔╝  █████╗        ███████║  ██║   ██║     ██║     ██║   ██║  ██╔██╗ ██║
  ██╔═══╝   ██╔══██╗  ██╔══╝        ██╔══██║  ██║   ██║     ██║     ██║   ██║  ██║╚██╗██║
  ██║       ██║  ██║  ███████╗      ██║  ██║  ╚██████╔╝     ██║     ╚██████╔╝  ██║ ╚████║
  ╚═╝       ╚═╝  ╚═╝  ╚══════╝      ╚═╝  ╚═╝   ╚═════╝      ╚═╝      ╚═════╝   ╚═╝  ╚═══╝
*/

/**
 * Selects Autonomous Mode based on the Limit Switch Input
 */

enum strat{
    SKILLS,
    PID,
};

double sideA = 1;
int i = 0;
strat Auton;

void increment(){
    i++;
    if(i>3){
      i=1;
    }
}

void pid_tune(double side){
    //chassis.moveToPoint(24*side,24,5000);

    // turn to face heading 90 with a very long timeout (PID TUNER)
    chassis.turnToHeading(90, 100000);
}

void skills(){
    belt(100);
    intake.move(-90);
    pros::delay(500);
    
    // first mogo
    chassis.moveToPoint(0,15,5000);
    //chassis.turnToPoint(-27,14.5,5000,{.forwards = false},true);
    chassis.turnToHeading(90, 5000);
    chassis.moveToPoint(-27,16,5000,{.forwards = false,.maxSpeed = 60},true);
    pros::delay(1000);
    toggleClamp();
    pros::delay(1000);

    belt(100);
    chassis.moveToPoint(-26, 30, 5000);
    chassis.moveToPoint(-26,45,5000);
    pros::delay(1500);
    belt(-100);
    pros::delay(1000);
    toggleSilly();
    chassis.moveToPoint(-26,30,5000,{.forwards = false},true);
    //chassis.turnToPoint(-53,60,5000);  
    
    // wall stake
    belt(100);
    chassis.moveToPoint(-53,59,1000);
    chassis.turnToHeading(270,5000);
    chassis.moveToPoint(-65,59,5000);
    //chassis.moveToPoint(-55, 59, 5000,{.forwards = false},true);
    chassis.moveToPoint(-70, 59, 5000);
    pros::delay(1000);
    toggleSilly();

    chassis.moveToPoint(-55,59,5000,{.forwards = false},true);
    chassis.moveToPoint(-55,58,5000);

    // corner 1
    chassis.turnToHeading(180,5000);
    chassis.moveToPoint(-55,35,5000);
    chassis.moveToPoint(-65,10,5000);
    chassis.moveToPoint(-55,30,5000,{.forwards = false},true);
    chassis.moveToPoint(-55,10,5000);
    chassis.moveToPoint(-55,-10,100);
    chassis.moveToPoint(-60,-28,100,{.forwards = false,.maxSpeed = 60});
    chassis.moveToPoint(-55,10,5000,{.forwards = false},true);
    chassis.turnToHeading(270,5000);
    //chassis.moveToPoint(-65,10,5000);

    chassis.moveToPoint(-70,20,5000);
    chassis.moveToPoint(-80,-15,1000,{.forwards = false}, true);
    pros::delay(500);
    belt(-100);
    toggleClamp();
    intake.move(90);
    pros::delay(500);
    intake.move(-90);
    belt(0);
    //chassis.moveToPoint(-24,24,5000);

    // move to corner 2
    chassis.moveToPoint(-60,60,5000);
    chassis.moveToPoint(-60,80,5000);
    chassis.moveToPoint(-33,90,5000);
    chassis.turnToHeading(180,5000);

    /*chassis.moveToPoint(-47,60,5000);
    chassis.moveToPoint(-47,80,5000);
    chassis.moveToPoint(-25,90,5000);
    chassis.turnToHeading(180,5000);
    chassis.moveToPoint(-25,115,5000,{.forwards = false}, true);
    pros::delay(500);
    toggleClamp();
    pros::delay(1000);
    chassis.moveToPoint(-75,115,1000);
    pros::delay(500);
    toggleRight();
    pros::delay(500);
    chassis.moveToPoint(-85,115,1000,{.forwards = false},true);
    belt(0);
    //toggleClamp();
    pros::delay(1000);
    chassis.moveToPoint(-55,100,5000);
    pros::delay(500);
    belt(100);
    pros::delay(500);
    belt(0);
    // alliance stake
    chassis.moveToPoint(-40,125,5000,{.forwards = false}, true);
    chassis.moveToPoint(-10,125,5000,{.forwards = false}, true);
    chassis.turnToHeading(180,5000);
    chassis.moveToPoint(-10,130,5000,{.forwards = false}, true);
    belt(100);
    pros::delay(4000);
    chassis.moveToPoint(-10,120,5000);
    chassis.turnToHeading(-50,5000);
    chassis.moveToPoint(0,110,5000,{.forwards = false}, true);
    toggleClamp();
    chassis.moveToPoint(-30,80,5000);
    */
}


void selectAuton(enum strat Auton, double side) {
    switch (Auton){
        case SKILLS:
            skills();
            break;
        case PID:
            pid_tune(side);
            break;
    }
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    chassis.setPose(0, 0, 0);

    detector.set_led_pwm(255);

    


    


    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            
            if(selector.get_value() == 1){
                increment();
            }
        
            if (i==1){
                Auton = SKILLS;
                pros::lcd::print(4, "Auton: %f", Auton);
            }
            else if (i==2){
                Auton = PID;
                pros::lcd::print(4, "Auton: %f", Auton);
            }
            else if (i==3) {
                pros::lcd::print(4, "TESTING: %f", Auton);
            }

            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

/*

 █████╗     ██╗   ██╗    ████████╗     ██████╗     ███╗   ██╗
██╔══██╗    ██║   ██║    ╚══██╔══╝    ██╔═══██╗    ████╗  ██║
███████║    ██║   ██║       ██║       ██║   ██║    ██╔██╗ ██║
██╔══██║    ██║   ██║       ██║       ██║   ██║    ██║╚██╗██║
██║  ██║    ╚██████╔╝       ██║       ╚██████╔╝    ██║ ╚████║
╚═╝  ╚═╝     ╚═════╝        ╚═╝        ╚═════╝     ╚═╝  ╚═══╝                                                           

*/

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy


/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    skills();

    //pid_tune(-sideA);

    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

/**
 
██████╗     ██████╗     ██╗    ██╗   ██╗    ███████╗    ██████╗ 
██╔══██╗    ██╔══██╗    ██║    ██║   ██║    ██╔════╝    ██╔══██╗
██║  ██║    ██████╔╝    ██║    ██║   ██║    █████╗      ██████╔╝
██║  ██║    ██╔══██╗    ██║    ╚██╗ ██╔╝    ██╔══╝      ██╔══██╗
██████╔╝    ██║  ██║    ██║     ╚████╔╝     ███████╗    ██║  ██║
╚═════╝     ╚═╝  ╚═╝    ╚═╝      ╚═══╝      ╚══════╝    ╚═╝  ╚═╝
                                                                
                                            
 */
 //i like the base because it moves -Jackson Best
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the chassis with curvature drive
        chassis.tank(leftY, rightY);

        pros::lcd::print(6, "Proximity value: %f", detector.get_proximity());

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            toggleClamp();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            toggleLeft();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            toggleRight();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            toggleSilly();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-100);
            belt(100);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) and not (detector.get_hue()<20)){
            intake.move(-100);
            belt(55);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            intake.move(100);
            belt(-100);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) and (detector.get_hue()<20)) {
            intake.move(100);
            belt(-55);
        }
        else{
            intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            intake.brake();
            belt(0);
        }

        /*if (detector.get_hue()<10) {
            pros::lcd::print(5, "Red:");
            pros::lcd::print(6, "Hue value: %f", detector.get_hue());
        }
        else if (detector.get_hue()>215 && detector.get_hue()<230){
            pros::lcd::print(5, "Blue:");
            pros::lcd::print(6, "Hue value: %f", detector.get_hue());
        }
        else {
            pros::lcd::print(5, " ");
            pros::lcd::print(6, "Hue value: %f", detector.get_hue());
        }*/
        

        // delay to save resources
        pros::delay(10);
    }
}