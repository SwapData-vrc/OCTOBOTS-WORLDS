#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
 int DRIVE_SPEED = 127;
 int TURN_SPEED = 127;
int SWING_SPEED = 110;

///
// Constants
///


void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 3.1, 200.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 20.1, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();

}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}


void unjam_task(){

  intake.move(127);
  int stallcount = 0;
  while (true) {
    pros::delay(15);
      if (intake.get_torque() > 1.05) {

        stallcount = stallcount + 1;

      }
      if (stallcount > 35) {

        stallcount = 0;
        intake.move(-127);
        pros::delay(100);
        intake.move(127);
      }
  
  }
// Credit to 20850V (Octobots)

}
// . . .
// Make your own autonomous functions here!
// . . .

void TwoGoalRight() {

/*chassis.pid_odom_set({{0_in, 36_in}, fwd, 110}); 
//intake on
chassis.pid_wait();

chassis.pid_turn_set(90_deg, 110); 
//matchloading 
chassis.pid_wait();

chassis.pid_odom_set({{0_in, 10_in}, fwd, 110}); 

pros::delay(300); 

chassis.pid_drive_set(-24_in, 127); 
chassis.pid_wait();

//outtake on

chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, 110, 30); 
//matchloading, outtake off
chassis.pid_wait();

chassis.pid_odom_set({{0_in, 10_in}, fwd, 110}); 
chassis.pid_wait();

chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 100); 
//intake on
chassis.pid_wait();

chassis.slew_drive_set(true);  
chassis.slew_drive_constants_set(6_in, 50);
chassis.pid_drive_set(16_in, 110); chassis.pid_wait();
//intake reverse
chassis.slew_drive_set(false);

chassis.pid_odom_set({{0_in, -20_in}, fwd, 127}); 
//intake off
chassis.pid_wait();

chassis.pid_turn_set(-315_deg, 110); 
chassis.pid_wait();

chassis.pid_drive_set(-18_in, 127); 
chassis.pid_wait();  */

}


//7 BLOCK RIGHT w/ PUSH
//15 secs
void OneGoalRight() { 
  DRIVE_SPEED = 127;
  TURN_SPEED = 127;
   intake.move(127);
   MiddleGoal.set_value(true);
  outtake.move(0);
 Snacky.set_value(true);
 chassis.pid_swing_set(ez::LEFT_SWING, 20_deg, 127, 70); 
 chassis.pid_wait_until(15_deg);
 Matchload.set_value(true);
chassis.pid_wait_quick_chain(); 
chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 127, 5);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::LEFT_SWING, 159_deg, 127, 57);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(24_in, 70, true);
chassis.pid_wait();
chassis.pid_turn_set(163_deg, TURN_SPEED);
pros::delay(300);
chassis.pid_drive_set(-29_in, 127, true);
chassis.pid_wait_until(-25_in);
outtake.move(127);
Snacky.set_value(false);
chassis.pid_wait();
pros::delay(1400);

chassis.pid_drive_set(10_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(110_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-9.8_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(158_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-20.6_in, 75, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(148_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();

 /*
//150 is matchload degree
chassis.pid_swing_set(ez::LEFT_SWING, 2_deg, 127, 1); 
chassis.pid_wait_quick_chain(); 
chassis.pid_drive_set(16_in, DRIVE_SPEED, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(true);
chassis.pid_drive_set(4_in, DRIVE_SPEED, true);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::LEFT_SWING, 130_deg, 127, 1); 
chassis.pid_wait_quick_chain(); 
chassis.pid_drive_set(27_in, DRIVE_SPEED, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(158_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(16.8_in, 70, true);
chassis.pid_wait();
pros::delay(67);
chassis.odom_xyt_set(0_in, 0_in, 158_deg);
intake.move(0);
chassis.pid_drive_set(-23.3_in, 127, true);
chassis.pid_wait_quick_chain();
intake.move(127);
outtake.move(127);
pros::Task unjam(unjam_task);
chassis.pid_drive_set(-4_in, 70, true);
chassis.pid_wait_quick_chain();
pros::delay(450);
Snacky.set_value(false);
chassis.pid_drive_set(10_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(110_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-10.2_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(158_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-20.6_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(148_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();

*/






    /*
MiddleGoal.set_value(true);
outtake.move(0);
intake.move(127);
chassis.pid_turn_set(8_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(26_in, 50);
chassis.pid_wait_quick_chain(); 
Matchload.set_value(true);
chassis.pid_turn_set(120_deg, 90);
chassis.pid_wait();
chassis.pid_drive_set(27_in, DRIVE_SPEED, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(165_deg, TURN_SPEED);
chassis.pid_wait();

intake.move(127);
chassis.pid_drive_set(23_in, DRIVE_SPEED-27); 
chassis.pid_wait();
pros::delay(60);
intake.move(127);

pros::delay(500);
chassis.pid_drive_set(-33.95_in, 120); 
chassis.pid_wait();
intake.move(-127);
outtake.move(-127);
pros::delay(300);
intake.move(127);
outtake.move(127);
pros::delay(2500);

chassis.pid_drive_set(7_in, DRIVE_SPEED);  
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(120_deg, TURN_SPEED);  
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-13.6_in, DRIVE_SPEED);  
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(170_deg, TURN_SPEED);  
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-12_in, DRIVE_SPEED);  
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(165_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_turn_set(0_deg, TURN_SPEED);
Snacky.set_value(false);


// 1. Combine Turn and Movement: Start intake while moving 
// Using a faster turn speed if your PID is tuned well 

chassis.pid_turn_set(30_deg, 127); 
intake.move(127); // Start intake immediately 

// 2. Drive to target position once 
chassis.pid_odom_set({{0_in, 36_in}, fwd, 127}); 
chassis.pid_wait(); 

// 3. Perform actions simultaneously 
// Lower matchloader while turning to the next heading 
Matchload.set_value(true);
chassis.pid_turn_set(90_deg, 110); 
chassis.pid_wait(); 

// 4. Tighten the swing turn 
// A swing turn is more accurate for small adjustments before backing up 
chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 110, 30); 
chassis.pid_wait(); 

pros::delay(300); 

// 6. Fast reverse to score 
chassis.pid_drive_set(-24_in, 127); 
chassis.pid_wait();

outtake.move(127); 
*/

}

//SKILLS
//1 min
void skills() {
 MiddleGoal.set_value(true);
 Snacky.set_value(true);
 Matchload.set_value(true);
chassis.pid_drive_set(37.4_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(90_deg, TURN_SPEED); 
chassis.pid_wait();
Matchload.set_value(true);
pros::delay(100);
intake.move(127);
outtake.move(0);
chassis.pid_drive_set(12.8_in, 40, true);
chassis.pid_wait();
pros::delay(1050);
intake.move(0);
chassis.pid_drive_set(-10.3_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(135_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_drive_set(-16_in, DRIVE_SPEED, true);
chassis.pid_wait();   
chassis.pid_turn_set(90_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_drive_set(-76_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(180_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_drive_set(14_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(-95_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-15.7_in, DRIVE_SPEED, true);  
chassis.pid_wait();
intake.move(-127);
outtake.move(-127);

pros::delay(200);
intake.move(127);
outtake.move(127);
pros::delay(3000);
outtake.move(0);
chassis.pid_drive_set(31.5_in, 80, true);  
chassis.pid_wait();
pros::delay(2000);
chassis.pid_drive_set(-32_in, DRIVE_SPEED, true);  
chassis.pid_wait();
intake.move(-127);
outtake.move(-127);

pros::delay(300);
intake.move(127);
outtake.move(127);
pros::delay(3000);

chassis.pid_drive_set(12_in, DRIVE_SPEED, true);  
chassis.pid_wait();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(97_in, DRIVE_SPEED, true);  
chassis.pid_wait();
outtake.move(0);
chassis.pid_turn_set(-90_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(16_in, 90, true);
chassis.pid_wait();
pros::delay(1500);


chassis.pid_drive_set(-6.7_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(-45_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_drive_set(-15.1_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(-90_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_drive_set(-76_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(0_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(83_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-9.3_in, DRIVE_SPEED, true);  
chassis.pid_wait();
intake.move(-127);
outtake.move(-127);

pros::delay(200);
intake.move(127);
outtake.move(127);
pros::delay(3000);
outtake.move(0);
chassis.pid_drive_set(30.6_in, 40, true);  
chassis.pid_wait();
pros::delay(1200);
chassis.pid_drive_set(-30.6_in, DRIVE_SPEED, true);  
chassis.pid_wait();
intake.move(-127);
outtake.move(-127);

pros::delay(200);
intake.move(127);
outtake.move(127);
pros::delay(3000);
Matchload.set_value(false);
chassis.pid_drive_set(17.3_in, 127, true);
chassis.pid_wait();
chassis.odom_xyt_set(0_in, 0_in, 90_deg);

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 80, 35);
  chassis.pid_wait();
  chassis.pid_turn_set(5_deg, 127);
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 127, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(true);
intake.move(127);
outtake.move(127);
  chassis.pid_drive_set(15_in, 127, true);
chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, 90, true);
chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(6_in, 40, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(false);
  chassis.pid_drive_set(-5_in, 90, true);
chassis.pid_wait();

pros::delay(10000);

}


//7 BLOCK LEFT w/ PUSH
//15 secs
void OneGoalLeft() {   DRIVE_SPEED = 127;
  TURN_SPEED = 127;
   intake.move(127);
   MiddleGoal.set_value(true);
  outtake.move(0);
 Snacky.set_value(true);
 chassis.pid_swing_set(ez::RIGHT_SWING, -20_deg, 127, 70); 
 chassis.pid_wait_until(-15_deg);
 Matchload.set_value(true);
chassis.pid_wait_quick_chain(); 
chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, 127, 5);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::RIGHT_SWING, -159_deg, 127, 57);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(24_in, 70, true);
chassis.pid_wait();
chassis.pid_turn_set(-163_deg, TURN_SPEED);
pros::delay(300);
chassis.pid_drive_set(-29_in, 127, true);
chassis.pid_wait_until(-25_in);
outtake.move(127);
Snacky.set_value(false);
chassis.pid_wait();
pros::delay(1400);

chassis.pid_drive_set(10_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-110_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-10.2_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-158_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-20.6_in, 75, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-148_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();

}

//Counter SAWP
//15 secs
void SAWP_counter(){


  pros::Task unjam(unjam_task);
  DRIVE_SPEED = 127;
  TURN_SPEED = 127;
 MiddleGoal.set_value(true);
 Snacky.set_value(true);
 Matchload.set_value(true);
chassis.pid_drive_set(15.3_in, DRIVE_SPEED, true);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 127, 15); 
chassis.pid_wait_quick_chain(); 
chassis.pid_drive_set(8.6_in, DRIVE_SPEED, true);
chassis.pid_wait();
outtake.move(30);
chassis.pid_drive_set(-28_in, DRIVE_SPEED, true);
chassis.pid_wait_until(-21_in);
outtake.move(127);
chassis.pid_wait();
pros::delay(100);
outtake.move(0);
Matchload.set_value(false);

chassis.pid_turn_set(180_deg, TURN_SPEED);
pros::delay(250);
/*
chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 127, 5); 
chassis.pid_wait_quick_chain(); 
chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 20); 
chassis.pid_wait_quick_chain(); 
chassis.pid_drive_set(47_in, 127, true);
chassis.pid_wait_quick_chain();

*/
chassis.pid_drive_set(66_in, 127, true);
pros::delay(2010);
Matchload.set_value(true);
chassis.pid_swing_set(ez::RIGHT_SWING, 137_deg, 127, 5); 
chassis.pid_wait_quick_chain(); 

Matchload.set_value(true);
chassis.pid_drive_set(29_in, 127, true);
pros::delay(1020);
chassis.pid_turn_set(90_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();
outtake.move(20);
chassis.pid_drive_set(-22.7_in, 127, true);
chassis.pid_wait_until(-15_in);
outtake.move(127);
chassis.pid_wait();


Matchload.set_value(true);
pros::delay(300);
outtake.move(0);
chassis.pid_drive_set(26.8_in, 100, true);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-5.3_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::RIGHT_SWING, 135_deg, 127, 29); 
chassis.pid_wait_quick_chain(); 
outtake.move(0);
chassis.pid_drive_set(-40_in, 127, true);
chassis.pid_wait_until(-27_in);
MiddleGoal.set_value(false);
outtake.move(-127);
chassis.pid_wait();
Matchload.set_value(false);
Snacky.set_value(false);
intake.move(127);
outtake.move(-127);
pros::delay(800);


chassis.pid_drive_set(13_in, 110, true);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 127, 10); 
chassis.pid_wait_quick_chain(); 
chassis.pid_drive_set(15_in, 110, true);
chassis.pid_wait();
chassis.pid_turn_set(-130_deg, TURN_SPEED); 
chassis.pid_wait();
/*
chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 127, 5); 
chassis.
outtake.move(0);
chassis.pid_drive_set(27.8_in, 100, true);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-4.4_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::RIGHT_SWING, 135_deg, 127, 29); 
chassis.pid_wait_quick_chain(); 
outtake.move(40);
chassis.pid_drive_set(-40_in, 127, true);
chassis.pid_wait_until(-36_in);
outtake.move(127);
chassis.pid_wait();
Matchload.set_value(false);
Snacky.set_value(false);
intake.move(127);
outtake.move(127);
pros::delay(800);


chassis.pid_drive_set(13_in, 110, true);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 127, 10); 
chassis.pid_wait_quick_chain(); 
chassis.pid_drive_set(15_in, 110, true);
chassis.pid_wait();
chassi

/*
  pros::Task unjam(unjam_task);
  DRIVE_SPEED = 127;
  TURN_SPEED = 127;
 MiddleGoal.set_value(true);
 Snacky.set_value(true);
 Matchload.set_value(true);
chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(90_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();

intake.move(127);
outtake.move(0);
chassis.pid_drive_set(12.4_in, 90, true);
chassis.pid_wait_quick_chain();
pros::delay(250);

chassis.pid_drive_set(-25.2_in, 127, true);
chassis.pid_wait_quick_chain();


intake.move(127);
outtake.move(127);
chassis.pid_drive_set(-3_in, 127, true);
chassis.pid_wait();
pros::delay(200);
outtake.move(0);
Matchload.set_value(false);
chassis.pid_drive_set(8_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(210_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(25_in, 127, true);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(4_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(34.7_in, 127, true);
chassis.pid_wait_quick_chain();

Matchload.set_value(true);
chassis.pid_drive_set(5_in, 127, true);
chassis.pid_wait_quick_chain();
*/
/*
chassis.pid_drive_set(1_in, 127, true);
chassis.pid_wait();
Matchload.set_value(false);
outtake.move(0);
chassis.pid_turn_set(185_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(4_in, 120, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(5_in, 110, true);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(15_in, 110, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(false);
chassis.pid_drive_set(21.5_in, 110, true);
chassis.pid_wait_quick_chain();
*/
/*
chassis.pid_turn_set(147_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();

Matchload.set_value(true);
chassis.pid_drive_set(25.5_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(90_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-8_in, 127, true);
chassis.pid_wait_quick_chain();
outtake.move(127);
chassis.pid_drive_set(-2_in, 127, true);
chassis.pid_wait();
Matchload.set_value(true);
pros::delay(100);
outtake.move(0);


chassis.pid_turn_set(90_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(27_in, 90, true);
chassis.pid_wait_quick_chain();
pros::delay(80);
chassis.pid_drive_set(-10.5_in, 127, true);

chassis.pid_wait_quick_chain();
chassis.pid_turn_set(135_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();

outtake.move(-127);

chassis.pid_drive_set(-42_in, 127, true);
chassis.pid_wait();

MiddleGoal.set_value(false);
intake.move(127);
outtake.move(127);
pros::delay(200);
intake.move(127);
outtake.move(-127);
pros::delay(1500);

*/
//LEFT 4+3
//15 secs

}
void TwoGoalL7() {
    pros::Task unjam(unjam_task);
  DRIVE_SPEED = 127;
  TURN_SPEED = 127;
 MiddleGoal.set_value(true);
 Snacky.set_value(true);
 Matchload.set_value(true);
chassis.pid_drive_set(16.2_in, DRIVE_SPEED, true);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, 127, 17); 
chassis.pid_wait_quick_chain(); 
chassis.pid_drive_set(5.3_in, DRIVE_SPEED, true);
chassis.pid_wait();
outtake.move(30);
chassis.pid_drive_set(-30_in, DRIVE_SPEED, true);
chassis.pid_wait_until(-24.3_in);
outtake.move(127);
chassis.pid_wait();
pros::delay(300);
outtake.move(0);
Matchload.set_value(false);

chassis.pid_turn_set(175_deg, TURN_SPEED);
pros::delay(300);

chassis.pid_drive_set(25_in, 127, true);
chassis.pid_wait();
Matchload.set_value(true);
chassis.pid_drive_set(-4.2_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-45_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-17.9_in, DRIVE_SPEED, true);
chassis.pid_wait_until(-15.1_in);
MiddleGoal.set_value(false);
outtake.move(-127);
chassis.pid_wait();
pros::delay(1300);

chassis.pid_drive_set(6_in, 110, true);
chassis.pid_wait_quick_chain();
MGDSCR.set_value(true);

chassis.pid_drive_set(-8_in, 100, true);
chassis.pid_wait_quick_chain();
Snacky.set_value(false);
Matchload.set_value(false);
MGDSCR.set_value(false);
chassis.pid_drive_set(10.5_in, 110, true);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 127, 10); 
chassis.pid_wait_quick_chain(); 
chassis.pid_drive_set(15_in, 110, true);
chassis.pid_wait();
chassis.pid_turn_set(130_deg, TURN_SPEED); 
chassis.pid_wait();


  /*
pros::Task unjam(unjam_task);
  DRIVE_SPEED = 127;
  TURN_SPEED = 127;
Snacky.set_value(false);
MiddleGoal.set_value(true);
Matchload.set_value(true);
//distance driven to first matchloader
chassis.pid_drive_set(31_in, DRIVE_SPEED, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-90_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();


intake.move(127);
outtake.move(0);
chassis.pid_drive_set(11_in, 80, true);
chassis.pid_wait();

pros::delay(1);
chassis.pid_drive_set(-20_in, 127, true);
chassis.pid_wait_quick_chain();
intake.move(-27);
outtake.move(-27);
chassis.pid_drive_set(-4.5_in, 127, true);
chassis.pid_wait_quick_chain();
intake.move(127);
outtake.move(127);

chassis.pid_drive_set(-3.8_in, 127, true);
chassis.pid_wait();
pros::delay(400);

Matchload.set_value(false);

chassis.pid_drive_set(15.5_in, 85, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(135_deg, TURN_SPEED); 
chassis.pid_wait_quick_chain();
outtake.move(0);
chassis.pid_drive_set(30_in, 90, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(true);
chassis.pid_turn_set(-40.5_deg, 90); 
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(-10.5_in, 90, true);
chassis.pid_wait_quick_chain();
MiddleGoal.set_value(false);
intake.move(-127);
outtake.move(127);
pros::delay(100);
chassis.pid_drive_set(-1_in, 100, true);
chassis.pid_wait_quick_chain();
intake.move(127);
outtake.move(-127);
pros::delay(1000);
chassis.pid_drive_set(7_in, 110, true);
chassis.pid_wait_quick_chain();
MGDSCR.set_value(true);

chassis.pid_drive_set(-9_in, 110, true);
chassis.pid_wait_quick_chain();

Matchload.set_value(false);
MGDSCR.set_value(false);
chassis.pid_drive_set(13_in, 110, true);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 127, 10); 
chassis.pid_wait_quick_chain(); 
chassis.pid_drive_set(15_in, 110, true);
chassis.pid_wait();
chassis.pid_turn_set(130_deg, TURN_SPEED); 
chassis.pid_wait();

*/
}





//SAWP WITH PUSH
//15 Secs
//Standard route

void SAWP_With_Push() {
  pros::Task unjam(unjam_task);
  DRIVE_SPEED = 127;
  TURN_SPEED = 127;
 MiddleGoal.set_value(true);
 Snacky.set_value(true);
chassis.pid_drive_set(9_in, 80, true);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(-20_in, 80, true);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(-21.7_in, 127, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(true);
intake.move(127);

chassis.pid_turn_set(-90_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(12.3_in, 85, true);
chassis.pid_wait();
chassis.pid_drive_set(-21.7_in, 127, true);
chassis.pid_wait_quick_chain();
intake.move(-27);
outtake.move(-27);
chassis.pid_drive_set(-1_in, 127, true);
chassis.pid_wait_quick_chain();
intake.move(127);
outtake.move(127);
chassis.pid_drive_set(-7_in, 127, true);
chassis.pid_wait();

outtake.move(0);
Matchload.set_value(false);

chassis.pid_drive_set(8_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(30_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(25_in, 127, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(true);
chassis.pid_drive_set(4_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(0_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(11_in, 127, true);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(8.7_in, 127, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(false);
chassis.pid_drive_set(15_in, 127, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(true);
chassis.pid_drive_set(10_in, 127, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-45_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(-3.2_in, 127, true);
chassis.pid_wait_quick_chain();
intake.move(-27);
outtake.move(127);
MiddleGoal.set_value(false);

chassis.pid_drive_set(-14.7_in, 90, true);
chassis.pid_wait_quick_chain();
intake.move(127);
outtake.move(-127);

pros::delay(600);
outtake.move(0);
MiddleGoal.set_value(true);
chassis.pid_drive_set(40.5_in, 120, true);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(8.5_in, 70, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-90_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(9.5_in, 127, true);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(-22_in, 127, true);
chassis.pid_wait_quick_chain();
intake.move(127);
outtake.move(127);


pros::delay(1000);
chassis.pid_drive_set(2_in, 127, true);
chassis.pid_wait();
chassis.pid_drive_set(-2_in, 127, true);
chassis.pid_wait();

}

void new_skills() {
  /*
  pros::Task unjam(unjam_task);
 MiddleGoal.set_value(true);
 Snacky.set_value(true);
 DRIVE_SPEED = 127;
  TURN_SPEED = 127;
  outtake.move(0);  
  intake.move(127);
  chassis.pid_swing_set(ez::RIGHT_SWING, -20_deg, 80, 40);
  chassis.pid_wait_quick_chain();

  Matchload.set_value(true);
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(-27);
  chassis.pid_drive_set(-12.4_in, 90, true);
  chassis.pid_wait();
  MiddleGoal.set_value(false);
  outtake.move(-20);
    chassis.pid_drive_set(-4_in, 90, true);
  chassis.pid_wait();
  intake.move(127);
  outtake.move(-127);
  pros::delay(1300);
  outtake.move(0);
  MiddleGoal.set_value(true);
 

  
chassis.pid_drive_set(51_in, 80, true);
chassis.pid_wait();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(9.5_in, 70, true);
chassis.pid_wait();
pros::delay(2500);
chassis.pid_drive_set(-10.3_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(135_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_drive_set(-16_in, DRIVE_SPEED, true);
chassis.pid_wait();   
chassis.pid_turn_set(180_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_drive_set(-70_in, 80, true);
chassis.pid_wait();
chassis.pid_turn_set(90_deg, TURN_SPEED); 
chassis.pid_wait();
chassis.pid_drive_set(11.5_in, DRIVE_SPEED, true);
chassis.pid_wait();
chassis.pid_turn_set(0_deg, TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-13.1_in, DRIVE_SPEED, true);  
chassis.pid_wait();
intake.move(-127);
outtake.move(-127);
chassis.pid_turn_set(8_deg, TURN_SPEED);
chassis.pid_wait();

pros::delay(200);
intake.move(127);
outtake.move(127);
pros::delay(3000);
outtake.move(0);
chassis.pid_drive_set(31.5_in, 80, true);  
chassis.pid_wait();
pros::delay(2000);
chassis.pid_drive_set(-32_in, 80, true);  
chassis.pid_wait();

intake.move(-127);
outtake.move(-127);
pros::delay(200);
intake.move(127);
outtake.move(127);
pros::delay(3000);
outtake.move(0);
*/
/*
intake.move(127);
Snacky.set_value(true);
chassis.pid_drive_set(7.3_in, 127, true);
chassis.pid_wait();
 Matchload.set_value(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 127, 57);
  chassis.pid_wait_quick_chain();
 
chassis.pid_drive_set(77_in, 100, true);
chassis.pid_wait();
Matchload.set_value(true);
  chassis.pid_drive_set(-22_in, 70, true);
chassis.pid_wait();

Matchload.set_value(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 127, 56);
  chassis.pid_wait();
  chassis.pid_turn_set(38_deg, 127);
  chassis.pid_wait();

   chassis.pid_drive_set(-39.4_in, 70, true);
chassis.pid_wait();
  intake.move(-45);
Matchload.set_value(true);  
MiddleGoal.set_value(false);
outtake.move(-10);
chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait();
  intake.move(127);
chassis.pid_drive_set(-6.3_in, 70, true);
chassis.pid_wait();
outtake.move(-127);
pros::delay(7000);
*/

Matchload.set_value(false);
intake.move(127);
Snacky.set_value(true);
chassis.pid_drive_set(12_in, 90, true);
pros::delay(1000);
chassis.pid_turn_set(45_deg, 127);
  chassis.pid_wait();
    chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 127, 40);
  chassis.pid_wait();

}




void park() {
  Matchload.set_value(false);
chassis.pid_drive_set(17.3_in, 127, true);
chassis.pid_wait();
chassis.odom_xyt_set(0_in, 0_in, 90_deg);

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 80, 35);
  chassis.pid_wait();
  chassis.pid_turn_set(5_deg, 127);
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 127, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(true);
intake.move(127);
outtake.move(127);
  chassis.pid_drive_set(10_in, 127, true);
chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, 70, true);
chassis.pid_wait_quick_chain();
Matchload.set_value(false);
  chassis.pid_drive_set(-5_in, 90, true);
chassis.pid_wait();

pros::delay(10000);


}




void wow() {
  pros::Task unjam(unjam_task);
 MiddleGoal.set_value(true);
 Snacky.set_value(true);
  intake.move(127);

 chassis.pid_swing_set(ez::LEFT_SWING, 39_deg, 90, 75);
pros::delay(870);
   chassis.pid_drive_set(30.5_in, 97, true);
  chassis.pid_wait();
  Matchload.set_value(true);
  chassis.pid_drive_set(-13.5_in, 127, true);
  chassis.pid_wait_quick_chain();

}