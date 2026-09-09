#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups

pros::MotorGroup leftMotors({13, -12, -11},pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-17, 19, 20}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
// leftMotors.set_gearing(pros::MotorGears::green, 2);
// rightMotors.set_gearing(pros::MotorGears::green, 2);
pros::Motor leftFront(-11, pros::MotorGearset::green);
// pros::Motor rightFront(-20, pros::MotorGearset::green);

// Inertial Sensoron port 10
pros::Imu imu(5);

pros::Distance dist(8);

pros::Motor leftCascade(10, pros::MotorGearset::blue);
pros::Motor rightCascade(18, pros::MotorGearset::blue);

pros::Motor intake(15, pros::MotorGearset::blue);

//pneumatics
pros::adi::DigitalOut claw('H', true);
pros::adi::DigitalOut swivle('G', false);

// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-9);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.35);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,// left motor group
                              &rightMotors, // right motor group
                              15, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(3, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            10, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.7, // proportional gain (kP)
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
lemlib::OdomSensors sensors(nullptr,//&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr,
                             // horizontal tracking wheel 2, set to nullptr as we don't have a second one
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

int autonSelection = 0;
const int NUM_AUTONS = 2;

void selectorTask(void*) {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Color Left (Right Side)");

    pros::lcd::register_btn0_cb([]() {
        autonSelection = (autonSelection - 1) % NUM_AUTONS;
        if (autonSelection == 0) pros::lcd::set_text(0, "Color Left (Right Side)");
        else if (autonSelection == 1) pros::lcd::set_text(0, "Color Right (Left Side)");
    });
    pros::lcd::register_btn2_cb([]() {
        autonSelection = (autonSelection + 1 + NUM_AUTONS) % NUM_AUTONS;
        if (autonSelection == 0) pros::lcd::set_text(0, "Color Left (Right Side)");
        else if (autonSelection == 1) pros::lcd::set_text(0, "Color Right (Left Side)");
    });

    while (true) pros::delay(100);
}

void initialize() {
    leftMotors.set_gearing(pros::MotorGears::green, 2);  // index 0 = front motor
    rightMotors.set_gearing(pros::MotorGears::green, 2);
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    claw.set_value(false);
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
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading4
            pros::lcd::print(3, "Distance: %d", dist.get());
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
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

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happyf
ASSET(path1_txt)

void resetX(){
    chassis.setPose(/**correct coord minus*/(dist.get()/** minus correct distance */), chassis.getPose().y, chassis.getPose().theta);
}

//ORDER
//.forwards
//.horizontalDrift
//.lead
//.maxSpeed
//.minSpeed
//.earlyExitRange

void colorLeft(){
    leftCascade.set_brake_mode(pros::MotorBrake::hold);
    rightCascade.set_brake_mode(pros::MotorBrake::hold);
    chassis.setPose(60,11,-225);
    chassis.moveToPose(41, 30, -225, 1000, {.forwards=false});
    leftCascade.move_velocity(-600);
    rightCascade.move_velocity(600);
    pros::delay(500);
    leftCascade.move_velocity(0);
    rightCascade.move_velocity(0);
    pros::delay(2000);
    leftCascade.move_velocity(600);
    rightCascade.move_velocity(-600);
    pros::delay(500);
    leftCascade.move_velocity(0);
    rightCascade.move_velocity(0);
    claw.set_value(true);
    // claw.set_value(false);
    // pros::delay(500);
    // leftCascade.move_velocity(600);
    // rightCascade.move_velocity(-600);
    // chassis.moveToPose(-12,0,-180,1000);
    // claw.set_value(true);
    // leftCascade.move_velocity(-600);
    // rightCascade.move_velocity(600);
    // chassis.moveToPose(-12, 23, 0, 1000, {}, false);
    // claw.set_value(false);
    // chassis.moveToPose(36, 13.5, 180, 3000);
    // leftCascade.move_velocity(600);
    // rightCascade.move_velocity(-600);
    // pros::delay(1000);
    // claw.set_value(true);
    // chassis.moveToPose(36, 24, 0, 1000, {}, false);
    // leftCascade.move_velocity(-600);
    // rightCascade.move_velocity(600);
    // claw.set_value(true);
}

void colorRight(){
    //opposite theta
    leftCascade.set_brake_mode(pros::MotorBrake::hold);
    rightCascade.set_brake_mode(pros::MotorBrake::hold);
    chassis.setPose(60,11,-225);
    chassis.moveToPose(41, 30, -225, 1000, {.forwards=false});
    leftCascade.move_velocity(-600);
    rightCascade.move_velocity(600);
    pros::delay(500);
    leftCascade.move_velocity(0);
    rightCascade.move_velocity(0);
    pros::delay(2000);
    claw.set_value(true);
    // claw.set_value(false);
    // pros::delay(500);
    // leftCascade.move_velocity(600);
    // rightCascade.move_velocity(-600);
    // chassis.moveToPose(-12,0,-180,1000);
    // claw.set_value(true);
    // leftCascade.move_velocity(-600);
    // rightCascade.move_velocity(600);
    // chassis.moveToPose(-12, 23, 0, 1000, {}, false);
    // claw.set_value(false);
    // chassis.moveToPose(36, 13.5, 180, 3000);
    // leftCascade.move_velocity(600);
    // rightCascade.move_velocity(-600);
    // pros::delay(1000);
    // claw.set_value(true);
    // chassis.moveToPose(36, 24, 0, 1000, {}, false);
    // leftCascade.move_velocity(-600);
    // rightCascade.move_velocity(600);
    // claw.set_value(true);
}

void easyAuto(){
    chassis.setPose(0,6.75,180);
    chassis.moveToPoint(0,24,1000, {.forwards=false});
    chassis.turnToHeading(-90,500);
    // leftCascade.move_velocity(-600);
    // rightCascade.move_velocity(600);
    // pros::delay(500);
    // leftCascade.move_velocity(0);
    // rightCascade.move_velocity(0);
    // chassis.moveToPoint(-22,24,1000);
    // leftCascade.move_velocity(600);
    // rightCascade.move_velocity(-600);
    // pros::delay(500);
    // leftCascade.move_velocity(0);
    // rightCascade.move_velocity(0);
    // claw.set_value(true);
    // chassis.moveToPoint(0,24,1000, {.forwards = false});
    // chassis.turnToHeading(-45,500);
    // chassis.moveToPoint(23,47,1000);
    // claw.set_value(false);
    // chassis.turnToHeading(22.5,500);
    // leftCascade.move_velocity(-600);
    // rightCascade.move_velocity(600);
    // pros::delay(750);
    // leftCascade.move_velocity(0);
    // rightCascade.move_velocity(0);
    // chassis.moveToPoint(24,24,1000);
    // claw.set_value(true);
}

void autonomous() {
    if (autonSelection == 0) colorLeft();
    else if (autonSelection == 1) colorRight();

    //pid tuning
    // chassis.setPose(0,0,0);
    // chassis.moveToPoint(0,24,1000);
}

/**
 * Runs in driver control
 */



float cubicDrive(float input, float scaling = 1.0f) {
    const float maxInput = 127.0f;
    return scaling * (input * input * input) / (maxInput * maxInput);
}

bool clawthing = false;
bool swivly = true;

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {

        float throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        float turn     = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        float leftPower  = cubicDrive(throttle) + cubicDrive(turn);
        float rightPower = cubicDrive(throttle) - cubicDrive(turn);

        if (leftPower  >  127.0f) leftPower  =  127.0f;
        if (leftPower  < -127.0f) leftPower  = -127.0f;
        if (rightPower >  127.0f) rightPower =  127.0f;
        if (rightPower < -127.0f) rightPower = -127.0f;

        leftMotors.move(leftPower);
        rightMotors.move(rightPower);

        // int macroNumber = 0;

        // if (macroNumber == 0){
        //     if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        //         leftCascade.move_relative(5*-360, 100);
        //         rightCascade.move_relative(4*360, 100);
        //         macroNumber = macroNumber + 1;
        //     }
        // }

        // else if (macroNumber != 0){
        //     if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        //         leftCascade.move_relative(4*-360, 100);
        //         rightCascade.move_relative(4*360, 100);
        //         macroNumber = macroNumber + 1;
        //     }
        //     else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        //         leftCascade.move_relative(4*360, 100);
        //         rightCascade.move_relative(4*-360, 100);
        //         macroNumber = macroNumber - 1;
        //     }
        // }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            leftCascade.move_relative(5*-360, 600);
            rightCascade.move_relative(4*360, 600);
        }

        //macro thingy, code first up, and button reset.

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            leftCascade.move_velocity(-600);
            rightCascade.move_velocity(600);
        }

        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move_velocity(-600);
        }

        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            intake.move_velocity(600);
        }


        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            leftCascade.move_velocity(600);
            rightCascade.move_velocity(-600);
        }

        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            clawthing = !clawthing;
            claw.set_value(clawthing);
        }

        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
            swivly = !swivly;
            swivle.set_value(swivly);
        }

        else {
            leftCascade.move_velocity(0);
            rightCascade.move_velocity(0);
            intake.move_velocity(0);
        }

    }
}