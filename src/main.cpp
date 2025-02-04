#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include <sys/_pthreadtypes.h>

#define RED 25
#define BLUE 75

pros::adi::Pneumatics clamp('H', false, false);
pros::adi::Pneumatics pto('F', false, false);

pros::Motor intake(17, pros::MotorGears::blue);

pros::MotorGroup lb({-1, 10});

// left motor group
pros::MotorGroup
    left_motor_group({-16, 15, -4},
                     pros::MotorGears::blue); // front, top, back
// right motor group
pros::MotorGroup
    right_motor_group({20, -19, 18},
                      pros::MotorGears::blue); // front, top, back

// color sort sensor
pros::Optical color_sensor(13);

// mogo clamp distance sensor
pros::Distance distance_sensor(1);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// const   pros::c::optical_rgb_s_t nehal_color = (); // color we throwing out

// LADY BROWN STUFF
const int lb_states = 4;
int states[lb_states] = {0, 460, 700, 2000};
int current_state = 0;
int target_state = 0;

void incrementState() {
  current_state++;
  if (current_state == lb_states) {
    current_state = 0;
  }
  target_state = states[current_state];
}

void lb_controller() {
  double kp = 0.5;
  double error =
      target_state - (lb.get_position(0) + abs(int(lb.get_position(1))) / 2);
  double volt = error * kp;
  lb.move(volt);
  pros::lcd::print(3, "State: %d", current_state);
  pros::lcd::print(4, "Target: %d", target_state);
  pros::lcd::print(5, "LB1: %f", lb.get_position(0));
  pros::lcd::print(6, "LB2: %f", lb.get_position(1));
}
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              11.25,                      // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2    // horizontal drift is 2 (for now)
);

// clamp with distance
void auto_clamp() {
  if (distance_sensor.get() < 15) { // mm
    clamp.set_value(true);
  } else {
    clamp.set_value(false);
  }
}

void intakeController() {

//   if (color_sensor.get_hue() <= RED && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
//     pros::delay(35);
//     intake.move_voltage(-12000);
//     pros::delay(150);
//   } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
//     intake.move_voltage(12000);
//   } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
//     intake.move_voltage(-12000);
//   } else {
//     intake.move_voltage(0);
//   }

  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    intake.move_voltage(12000);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    intake.move_voltage(-12000);
  } else {
    intake.move_voltage(0);
  }
}

// imu
pros::Imu imu(15);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(11);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(2);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_2, 2.5);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_2, -0.125);

// odometry settings
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(9,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       40,  // derivative gain (kD)
                       0,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(5,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       45,  // derivative gain (kD)
                       0,   // anti windup
                       2,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       5,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

bool ptotoggled;
// initialize function. Runs on program startup
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  lb.set_zero_position(0);
  intake.set_brake_mode(pros::MotorBrake::coast);
  color_sensor.set_integration_time(3);
  color_sensor.set_led_pwm(25);
  left_motor_group.set_brake_mode(pros::MotorBrake::coast);
  right_motor_group.set_brake_mode(pros::MotorBrake::coast);
  ptotoggled = false;
  pros::Task screen_task([&]() {
    while (true) {
      intakeController();
      lb_controller();

      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // delay to save resources
      pros::delay(10);
    }
  });
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
void autonomous() { chassis.setPose(0, 0, 180); }

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

  while (true) {
    // get left y and right x positions
    int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turning = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.arcade(throttle, turning, true);

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      clamp.toggle();
      controller.rumble(".");
    }

    if (distance_sensor.get() < 25) { // mm
        controller.rumble("...");
  }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      pto.toggle();
    }

    // if (color_sensor.get_hue() <= 25) {
    //     intake.brake();
    //     intake.move_voltage(-11000);
    // }
    // else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    //     intake.move_voltage(11000);
    // } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    //     intake.move_voltage(-11000);
    // } else {
    //     intake.move_voltage(0);
    // }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      incrementState();
    }

    // delay to save resources
    pros::delay(10);
  }
}
//testing
