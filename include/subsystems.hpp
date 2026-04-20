#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
inline pros::Motor intake(17);
inline pros::Motor outtake(15);

inline pros::adi::DigitalOut Matchload('A');
inline pros::adi::DigitalOut Snacky('B');
inline pros::adi::DigitalOut MiddleGoal('C');
inline pros::adi::DigitalOut MGDSCR('D');
