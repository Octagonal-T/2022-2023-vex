#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <constants.h>
#include <subtasks.h>

bool leftMotorMoving = true;
bool rightMotorMoving = true;
bool previousIntakeCommand = false;
bool driverControl = false;
int intakeSecondsHolding = 0;

int updateControllerScreen(){
  while(driverControl){
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print("Flywheel: %dV, %dW", floor(Flywheel.voltage(vex::volt)), floor(Flywheel.power(vex::watt)));
    Controller.Screen.newLine();
    Controller.Screen.print("Intake: %dV, %dW", floor(Intake.voltage(vex::volt)), floor(Intake.power(vex::watt)));
    Controller.Screen.newLine();
    int seconds = floor(expansionSeconds / 50);
    int mins = floor(seconds / 60);
    seconds = seconds - (mins*60);
    Controller.Screen.newLine();
    Controller.Screen.print("Game time: %d:%d", mins, seconds);
    vex::task::sleep(250);
  }
  return 1;
}
void drivercontrol(){
  driverControl = true;
  Controller.ButtonL2.pressed(toggleFlywheel);
  Controller.ButtonL1.pressed(toggleIndexer);
  Controller.ButtonRight.pressed(expansion);
  vex::task controllerScreen(updateControllerScreen);
  while(true){
    double leftJoystick = abs(Controller.Axis3.position()) > 5 ? Controller.Axis3.position() : 0;
    double rightJoystick = abs(Controller.Axis1.position()) > 5 ? Controller.Axis1.position() : 0;

    double leftSideSpeed = leftJoystick + rightJoystick;
    double rightSideSpeed = leftJoystick - rightJoystick;

    if (fabs(leftSideSpeed) < 5) {
      if (leftMotorMoving) {
        LeftMotors.stop();
        leftMotorMoving = false;
      }
    } else {
      leftMotorMoving = true;
    }
    if (fabs(rightSideSpeed) < 5) {
      if (rightMotorMoving) {
        RightMotors.stop();
        rightMotorMoving = false;
      }
    } else {
      rightMotorMoving = true;
    }
    if (leftMotorMoving) {
      LeftMotors.spin(vex::reverse, leftSideSpeed, vex::percent);
    }
    if (rightMotorMoving) {
      RightMotors.spin(vex::reverse, rightSideSpeed, vex::percent);
    }

    if(Controller.ButtonR1.pressing()){
      if(!previousIntakeCommand){
        toggleIntake();
      }else{
        intakeSecondsHolding++;
        if(intakeSecondsHolding == 25){
          Intake.spin(vex::reverse, 100, vex::percent);
          intakeMoving = 2;
        }
      }
      previousIntakeCommand = true;
    }else{
      if(intakeMoving == 2){
        Intake.stop();
        intakeMoving = 0;
      }
      previousIntakeCommand = false;
      intakeSecondsHolding = 0;
    }
    expansionSeconds++;
    vex::task::sleep(20);
  }
  driverControl = false;
}