#ifndef DUPLO_TRAIN_TESTS_H
#define DUPLO_TRAIN_TESTS_H

#include "DuploHub.h"

void timingMotor(DuploHub& duploHub, bool& demoRunning, int& testCount);
void timingSound(DuploHub& duploHub, bool& demoRunning, int& testCount);
void timingLED(DuploHub& duploHub, bool& demoRunning, int& testCount);
void duploHubDemo(DuploHub& duploHub, bool& demoRunning, unsigned long& lastDemoStep, int& demoStep);

#endif // DUPLO_TRAIN_TESTS_H
