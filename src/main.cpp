#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cstdio>
#include <utility>
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor declarations - DRIVETRAIN TAKEN CARE OF IN MOTORGROUP DECLARATIONS
// Real declarations START HERE -------------------------------------------------------------------------------------------------<
pros::Motor motorTopIntake(-14);
pros::Motor motorTopIntake2(-16);
pros::Motor motorMidIntake3(-17);
pros::Motor motorMidIntake4(-8);
pros::Motor motorBottomIntake5(-11);
pros::Motor motorBottomIntake6(-12);
// naming for intake goes: look at the back of the bot. 
// Then go: high-->low, back-->front (of the bot), left--> right.
// The priority goes in that order from 1-->6

// Notebook: I can't put motors in a motor group with different gearsets as neccesitated by the mix of 11 and 5.5 watt motors in our drivetrain. Because of this, I run into a problem:
// Potential fix: ignore issue since incorrectly describing a motor's gear ratio to pros will only mess with motor encoder values, which we will not be using anyways.
pros::MotorGroup leftMotors({-1,-2,+3}); // left motor group - ports 1, 2, 3 (reversed)
pros::MotorGroup rightMotors({4,5,-6}); // right motor group - ports 4, 5, 6 (reversed)
pros::MotorGroup allDriveMotors({1,2,-3,4,5,-6}); //all the drivetrainmotors, with ports 3 and 6 reversed
pros::MotorGroup allIntakeMotors({-14,-16,-17,-8,-11,-12}); //all the motors in the intake
// configuring the drivetrain
//PNEUMATICS
pros::adi::DigitalOut descore('E'); //Pneumatics on port A



// Inertial Sensor on port 10
pros::Imu imu(10);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(19);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(20);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -7.83);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.14);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.125, //12.125 inch track width  
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              600, // drivetrain rpm is 600
                              7 // horizontal drift is 2. If we had traction wheels, it would have been 8
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
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/*
Spins the intake for autonomous for match loading. Balls go from intake to storage.
*/
struct MoveParams {
    double timeout;
};
void Matchload(void *param) {
    MoveParams* p = (MoveParams*)param;
    motorTopIntake.move(127); 
    motorTopIntake2.move(127); 
    motorMidIntake3.move(127);
    motorMidIntake4.move(-127); //reverse
    motorBottomIntake5.move(127);
    motorBottomIntake6.move(127);
    pros::delay(p->timeout);
    allIntakeMotors.move(0);
}
void scoreMidGoal(void *param) {
    MoveParams* p = (MoveParams*)param;
    motorTopIntake.move(127); 
    motorTopIntake2.move(-127); //reverse
    motorMidIntake3.move(127);
    motorMidIntake4.move(-127); //reverse
    motorBottomIntake5.move(-127);//reverse
    motorBottomIntake6.move(127);
    pros::delay(p->timeout);
    allIntakeMotors.move(0);
}
void scoreTopGoal(void *param) {
    MoveParams* p = (MoveParams*)param;
    motorTopIntake.move(-127); //reverse
    motorTopIntake2.move(127); 
    motorMidIntake3.move(127);
    motorMidIntake4.move(-127); //reverse
    motorBottomIntake5.move(-127);//reverse
    motorBottomIntake6.move(127);
    pros::delay(p->timeout);
    allIntakeMotors.move(0);

}
void scoreLowGoal(void *param) {
    MoveParams* p = (MoveParams*)param;
    motorBottomIntake6.move(-127); 
    //will always be in reverse:
    motorBottomIntake5.move(-127);
    //rest of the motors:
    motorTopIntake.move(0); 
    motorTopIntake2.move(0); //reverse
    motorMidIntake3.move(0); //reverse
    motorMidIntake4.move(-127); //reverse
    pros::delay(p->timeout);
    allIntakeMotors.move(0);
}
void pneumatics_task(void* param) {
    bool descore_state = false; // Track state

    while (true) {
        // Toggle on button press
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            descore_state = !descore_state; // Flip the state
            descore.set_value(descore_state);
        }
        
        pros::delay(20); // Faster response
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
            // delay to save resources/ the fureakjng program
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

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy
//double functions here


/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // this is for RED side
    pros::lcd::print(4,"AUSTO");
    chassis.setPose(-60.887, 17.025, 90);
    MoveParams matchload_params = {5000.0}; // Run for 2000ms
    pros::Task matchload_task_handle(Matchload, &matchload_params);
    chassis.moveToPoint(-50.162, 17.131, 2000, {.maxSpeed=70});

}

/**
 * Runs in driver control
 */
void opcontrol() {
   
    // pneumatics controller
    pros::Task pneumatics_task_handle(pneumatics_task);
    // loop to continuously update ALL motors
    // WE NEED TO THINK OF HOW TO MAKE DRIVING SMOOTHER --> resolved because lemlib already has expothrottling
    // THE MAIN PROBLEM IN OUR OPINION IS THAT THE INPUT RESOLUTION OF THE CONTROLLERS IS WAY TOO LOW  --> resolved, look above
    while (true) {
        //*DIVE TRAIN*
        // get joystick positions   
        int leftY   = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.curvature(leftY,rightX);
        // delay to save resources
        pros::delay(20);
        

        
        //*INTAKE*-----------------------------------------------<Added all sections of intake scoring 9/29/25>

        // TOP GOAL
        if (controller.get_digital(DIGITAL_L1)){
            //stays constant in all except for low goal
            motorBottomIntake6.move(127); //this one stays constant except for rev. (Low goal)
            // SO we can outtake from storage:
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
                motorBottomIntake5.move(-127);
                motorBottomIntake6.move(1);
            }
            else {
                motorBottomIntake5.move(127);
                motorBottomIntake6.move(127);
            }
            //rest of the motors
            motorTopIntake.move(-127); //reverse
            motorTopIntake2.move(127); 
            motorMidIntake3.move(127);
            motorMidIntake4.move(-127); //reverse



            
            
        }
            //STORAGE 
        else if (controller.get_digital(DIGITAL_R2)) {
            
            motorBottomIntake6.move(127);
            // SO we can outtake from storage:
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
                motorBottomIntake5.move(-127);
            }
            else {
                motorBottomIntake5.move(127);
            }
            //rest of the motors
            motorTopIntake.move(127); 
            motorTopIntake2.move(127); 
            motorMidIntake3.move(127);
            motorMidIntake4.move(-127); //reverse 
            
        }
        //LOW GOAL
        else if (controller.get_digital(DIGITAL_R1)) {
           // stays constant in all except for low goal --> THIS IS LOW GOAL
            motorBottomIntake6.move(-127); 
            //will always be in reverse:
            motorBottomIntake5.move(-127);
            //rest of the motors:
            motorTopIntake.move(0); 
            motorTopIntake2.move(0); //reverse
            motorMidIntake3.move(0); //reverse
            motorMidIntake4.move(-127); //reverse
        }
        //MID GOAL
        else if (controller.get_digital(DIGITAL_L2)) {
            // stays constant in all except for low goal
            
            // SO we can outtake from storage:
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
                motorBottomIntake5.move(-127);
                motorBottomIntake6.move(1);
            }
            else {
                motorBottomIntake5.move(127);
                motorBottomIntake6.move(127);
            }
            //rest of the motors
            motorTopIntake.move(127); 
            motorTopIntake2.move(-127); //reverse
            motorMidIntake3.move(127);
            motorMidIntake4.move(-127); //reverse
        }
        else {
            allIntakeMotors.move(0); //stops all motors when nothing is pressed

        
        }


    }
} 
