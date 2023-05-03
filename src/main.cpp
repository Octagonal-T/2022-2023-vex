/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       octag                                                     */
/*    Created:      2/25/2023, 12:15:03 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <auton.h>
#include <driver.h>
#include <constants.h>

int main() {
  preAuton();
  // startAutonomous();
  Competition.autonomous(startAutonomous);
  Competition.drivercontrol(drivercontrol);
}
