#include <vex.h>
#ifndef TASKS_H_
#define TASKS_H_

extern int intakeMoving;

int launcherPID();
void toggleFlywheel();
void toggleIndexer();
void toggleIntake();
void expansion();

#endif