#include "main.h"
#include <fstream>
#include <sstream>
#include <string>
#include "ramsete.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
    initRamsete(2.0, 0.7);
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

pros::MotorGroup left_motors({19});
pros::MotorGroup right_motors({5});
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::OLD_4, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, nullptr);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, sensors// odometry sensors
);
void opcontrol() {
    lemlib::Pose currentPose = chassis.getPose();
    lemlib::Pose targetPose(24, 24, 90); // Manually setting target position (x, y, theta)

    double vd = 1.0;  // Desired linear velocity
    double wd = 0.5;  // Desired angular velocity

    while (true) {
        currentPose = chassis.getPose();

        // Calculate distance to target
        double distanceError = std::sqrt(
            std::pow(targetPose.x - currentPose.x, 2) + 
            std::pow(targetPose.y - currentPose.y, 2)
        );
		
        // Calculate angle error (normalize to -180 to 180)
        double angleError = targetPose.theta - currentPose.theta;
        while (angleError > 180) angleError -= 360;
        while (angleError < -180) angleError += 360;

        // Check if we've reached the target (within tolerance)
        if (distanceError < 1.0 && std::abs(angleError) < 5.0) {
            break; // Stop when the target is reached
        }

        // Calculate wheel velocities using Ramsete
        auto [leftVelocity, rightVelocity] = calculate(currentPose, targetPose, vd, wd);

        // Apply velocities to motors
        left_motors.move_velocity(leftVelocity);
        right_motors.move_velocity(rightVelocity);

        pros::delay(20);  // 20ms control loop (50Hz)
    }

    // Stop motors when target is reached
    left_motors.move_velocity(0);
    right_motors.move_velocity(0);
}
