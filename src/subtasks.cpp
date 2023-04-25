#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <constants.h>

bool flywheelEngaged = false;
bool confirmExpand = false;
int intakeMoving = 0; //0 = not moving, 1 = forward, 2 = reverse
int flywheelSpeed = 0;

void toggleFlywheel(){
  if(flywheelEngaged && flywheelSpeed == 9){
    Flywheel.stop();
    flywheelSpeed = 0;
    flywheelEngaged = false;
  }else if(flywheelSpeed == 12){
    Flywheel.spin(vex::forward, 9, vex::volt);
    flywheelSpeed = 9;
  }else{
    Flywheel.spin(vex::forward, 12, vex::volt);
    flywheelSpeed = 12;
    flywheelEngaged = true;
  }
}
void toggleIndexer(){
  Indexer.setVelocity(50, vex::percent);
  Indexer.spinFor(-0.2, vex::rev, true);
  Indexer.spinFor(0.2, vex::rev, true);  
}
void toggleIntake(){
  if(intakeMoving == 1){
    Intake.stop();
    intakeMoving = 0;
  }else{
    Intake.spin(vex::forward, 100, vex::percent);
    intakeMoving = 1;
  }
}
void expansion(){
  if(expansionSeconds >= 5250 || confirmExpand){
    Expansion.setVelocity(100, vex::percent);
    Expansion.spinFor(0.5, vex::rev, true);
  }else{
    Controller.Screen.clearScreen();
    Controller.Screen.print("AUTONOMOUS NOT REACHED\nCLICK AGAIN TO EXPAND PREMATURELY");
    confirmExpand = true;
  }
}