#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <constants.h>
#include <subtasks.h>
#include <auton.h>

int routine;
PIDVariables lateralPID;
PIDVariables rotationPID;
bool movementFinished;
bool engageLateral = true;
bool engageRotational = true;
bool autoEngaged = false;
int confirmSeconds;
int maxLeft = 100;
int maxRight = 100;
int directional = -1;

void driveTo(double lateral, double heading, int maxLeft2=100, int maxRight2=100, int directional2=-1){
  lateralPID.target = (lateral/7.06858347)*360;
  rotationPID.target = heading;
  maxLeft = maxLeft2;
  maxRight = maxRight2;
  directional = directional2;
  if(lateral == 0){
    engageLateral = false;
  }else{
    engageLateral = true;
  }
  if(heading == 0){
    engageRotational = false;
  }else{
    engageRotational = true;
  }
  movementFinished = false;
}
int drivePID(){
  while(autoEngaged){
    double pos = Encoder.rotation(vex::degrees);
    double currentHeading = Inertial.heading(vex::degrees);
    lateralPID.error = pos - lateralPID.target;

    rotationPID.error = rotationPID.target - currentHeading;

    if(fabs(rotationPID.error) > 180) rotationPID.error = currentHeading - rotationPID.target;
    if(fabs(rotationPID.error) > 180) rotationPID.error = currentHeading +360-rotationPID.target;
    if(fabs(rotationPID.error) > 180) rotationPID.error = -360 + rotationPID.target - currentHeading;
    
    if((fabs(lateralPID.error) > 35 && engageLateral) || (fabs(rotationPID.error) > 2 && engageRotational)){
      if(confirmSeconds > 0){
        confirmSeconds=0;
        directional = -1;
      }
      movementFinished = false;
      lateralPID.derivative = lateralPID.error - lateralPID.lastError;
      rotationPID.derivative = rotationPID.error - rotationPID.lastError;
      lateralPID.integral+=lateralPID.error;
      rotationPID.integral += rotationPID.error;
      double lateralVelocity = (lateralPID.error * lateralPID.kP + lateralPID.derivative * lateralPID.kD + lateralPID.integral * lateralPID.kI);
      double turnVelocity = fabs(rotationPID.error * rotationPID.kP + rotationPID.derivative * rotationPID.kD + rotationPID.integral * rotationPID.kI);
      
      double leftMotorsVelocity;
      double rightMotorsVelocity;
      if(engageRotational){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1,1);
        Controller.Screen.print(currentHeading);
        Controller.Screen.newLine();
        Controller.Screen.print(rotationPID.target);
        Controller.Screen.newLine();
        Controller.Screen.print(rotationPID.error);
        if(directional == -1){
          if(rotationPID.error > 0){
            leftMotorsVelocity = -turnVelocity;
            rightMotorsVelocity = turnVelocity;
          }else{
            leftMotorsVelocity = turnVelocity;
            rightMotorsVelocity = -turnVelocity;
          }
        }else{
          if(directional == 0){
            leftMotorsVelocity = -turnVelocity;
            rightMotorsVelocity = turnVelocity;
          }else{
            leftMotorsVelocity = turnVelocity;
            rightMotorsVelocity = -turnVelocity;
          }
        }
      }else{
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1,1);
        Controller.Screen.print(pos);
        Controller.Screen.newLine();
        Controller.Screen.print(lateralPID.target);
        Controller.Screen.newLine();
        Controller.Screen.print(lateralPID.error);
        leftMotorsVelocity = (lateralVelocity);
        rightMotorsVelocity = (lateralVelocity);
      }

      if(leftMotorsVelocity > maxLeft) leftMotorsVelocity = maxLeft;
      else if(leftMotorsVelocity < -maxLeft) leftMotorsVelocity = -maxLeft;
      if(rightMotorsVelocity > maxRight) rightMotorsVelocity = maxRight;
      else if(rightMotorsVelocity < -maxRight) leftMotorsVelocity = -maxRight;

      LeftMotors.spin(vex::forward, leftMotorsVelocity, vex::percent);
      RightMotors.spin(vex::forward, rightMotorsVelocity, vex::percent);

      lateralPID.lastError = lateralPID.error;
      rotationPID.lastError = rotationPID.error;
    }else{
      confirmSeconds++;
      if(confirmSeconds==15){
        confirmSeconds = 0;
        movementFinished = true;
        Encoder.resetRotation();
        lateralPID.target = 0;
        lateralPID.lastError = 0;
        rotationPID.lastError = 0;
        engageLateral = false;
        engageRotational = false;
        LeftMotors.stop();
        RightMotors.stop();
        maxLeft = 100;
        maxRight = 100;
      }
    }
    vex::task::sleep(20);
  }
  return 1;
}

void preAuton(){
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Calibrating inertial");
  Inertial.calibrate();
  while(Inertial.isCalibrating()){
    vex::task::sleep(100);
  }
  Encoder.resetRotation();
  Inertial.resetHeading();
  Controller.Screen.newLine();
  Controller.Screen.print("Finished calibrating");
  Controller.Screen.newLine();
  lateralPID.kP = 0.06;
  lateralPID.kI = 0;
  lateralPID.kD = 0.005;

  rotationPID.kP = 0.4;
  // rotationPID.kI = 0.01;
  rotationPID.kD = 0.098;

  movementFinished = true;
  confirmSeconds = 0;

  Controller.Screen.print("Set PID variables");
  vex::task::sleep(1000);
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Left: left routine");
  Controller.Screen.newLine();
  Controller.Screen.print("Right: right routine");
  Controller.Screen.newLine();
  Controller.Screen.print("X: no routine");
  bool flag = false;
  for (int i = 0; i < 500; i++){
    if(Controller.ButtonLeft.pressing()){
      routine = 1;
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Left selected");
      flag = true;
      break;
    }else if(Controller.ButtonRight.pressing()){
      routine = 2;
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Right selected");
      flag = true;
      break;
    }else if(Controller.ButtonX.pressing()){
      routine = 0;
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("No routine");
      flag = true;
      break;
    }else if(Controller.ButtonDown.pressing()){
      routine = 3;
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Test routine");
      flag = true;
      break;

    }
    vex::task::sleep(10);
  }
  if(!flag){
    routine = 0;
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("No routine");
  }
  vex::task::sleep(1000);
  Controller.Screen.clearScreen();
  Brain.Screen.drawImageFromFile("dougal.png", 0, 0);
}
void startAutonomous(){
  autoEngaged = true;
  vex::task drivePIDTask(drivePID);
  if(routine == 1){ // left
    autoEngaged = false;
    Intake.spin(vex::reverse, 100, vex::percent);
    LeftMotors.spin(vex::reverse, 100, vex::percent);
    RightMotors.spin(vex::reverse, 100, vex::percent);
    vex::task::sleep(300);
    Intake.stop();
    LeftMotors.stop();
    RightMotors.stop();
    Encoder.resetRotation();
    autoEngaged = true;
    vex::task drivePIDTask(drivePID);
    driveTo(-5, 0);
    waitUntil(movementFinished);
    Flywheel.spin(vex::forward, 12, vex::volt);
    driveTo(0, 353, 100, 100, 1);
    waitUntil(movementFinished);
    vex::task::sleep(750);
    toggleIndexer();
    vex::task::sleep(1500);
    toggleIndexer();
    vex::task::sleep(50);
    driveTo(0, 235, 100, 100);
    Flywheel.stop();
    waitUntil(movementFinished);
    Intake.spin(vex::forward, 100, vex::percent);
    driveTo(45, 0, 75, 75);
    waitUntil(movementFinished);
  }else if(routine == 2){ // right
    driveTo(20, 0);
    waitUntil(movementFinished);
    driveTo(0, 90, 100, 100, 0);
    waitUntil(movementFinished);
    Intake.spin(vex::reverse, 100, vex::percent);
    LeftMotors.spin(vex::reverse);
    RightMotors.spin(vex::reverse);
    vex::task::sleep(300);
    LeftMotors.stop();
    RightMotors.stop();
    Intake.stop();
    driveTo(-5, 0);
    waitUntil(movementFinished);
    Flywheel.spin(vex::forward, 11.5, vex::volt);
    driveTo(0, 99, 100, 100, 0);
    waitUntil(movementFinished);
    vex::task::sleep(750);
    toggleIndexer();
    vex::task::sleep(1500);
    toggleIndexer();
  }else if(routine == 3){ // test
    driveTo(100, 0);
    waitUntil(movementFinished);
    driveTo(0, 180);
    waitUntil(movementFinished);
    driveTo(100, 0);
    waitUntil(movementFinished);
  }
  autoEngaged = false;
}