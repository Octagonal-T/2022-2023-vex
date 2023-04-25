#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <constants.h>

int expansionSeconds = 0;
vex::brain Brain;
vex::competition Competition;
vex::controller Controller(vex::primary);

vex::motor LeftMotor1(vex::PORT11, vex::ratio36_1, false);
vex::motor LeftMotor2(vex::PORT13, vex::ratio36_1, false);
vex::motor RightMotor1(vex::PORT12, vex::ratio36_1, true);
vex::motor RightMotor2(vex::PORT14, vex::ratio36_1, true);
vex::motor Flywheel(vex::PORT15, vex::ratio6_1, true);
vex::motor Indexer(vex::PORT16, vex::ratio18_1, false);
vex::motor Intake(vex::PORT17, vex::ratio18_1, true);
vex::motor Expansion(vex::PORT18, vex::ratio18_1, false);

vex::inertial Inertial(vex::PORT20);
vex::encoder Encoder(Brain.ThreeWirePort.G);

vex::motor_group LeftMotors(LeftMotor1, LeftMotor2);
vex::motor_group RightMotors(RightMotor1, RightMotor2);