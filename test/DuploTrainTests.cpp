#include "DuploTrainTests.h"
#include "debug.h"

void timingMotor(DuploHub& duploHub, bool& demoRunning, int& testCount) {
    if (duploHub.isConnected() && (testCount < 5)) {
        unsigned long currentMillis = millis();
        DEBUG_LOG("TrainController: setMotorSpeed called at: %lu", currentMillis);
        DEBUG_LOG("TrainController Demo: Motor forward (speed 35)");
        duploHub.setMotorSpeed(35);
        delay(5000);
        testCount++;
    }
}

void timingSound(DuploHub& duploHub, bool& demoRunning, int& testCount) {
    if (duploHub.isConnected() && (testCount < 10)) {
        unsigned long currentMillis = millis();
        DEBUG_LOG("TrainController: playSound called at: %lu", currentMillis);
        DEBUG_LOG("TrainController Demo: Playing sound %d", 3 + testCount);
        duploHub.playSound((DuploEnums::DuploSound) 3 + testCount);
        delay(5000);
        testCount = testCount + 2;
    }
}

void timingLED(DuploHub& duploHub, bool& demoRunning, int& testCount) {
    if (duploHub.isConnected() && demoRunning && testCount >= 0 && testCount <= 10) {
        unsigned long currentMillis = millis();
        DEBUG_LOG("TrainController: setLEDColor called at: %lu", millis());
        DEBUG_LOG("TrainController Demo: Setting LED to color %d", testCount);
        duploHub.setLedColor((DuploEnums::DuploColor)(testCount));
        delay(5000);
        testCount++;
    }
}

void duploHubDemo(DuploHub& duploHub, bool& demoRunning, unsigned long& lastDemoStep, int& demoStep) {
    if (duploHub.isConnected() && demoRunning) {
        unsigned long currentTime = millis();
        if (currentTime - lastDemoStep >= 5000) {
            lastDemoStep = currentTime;
            switch (demoStep) {
                case 0:
                    DEBUG_LOG("TrainController Demo: Setting LED to GREEN");
                    duploHub.setLedColor(DuploEnums::DuploColor::GREEN);
                    break;
                case 1:
                    DEBUG_LOG("TrainController Demo: Setting LED to RED");
                    duploHub.setLedColor(DuploEnums::DuploColor::RED);
                    break;
                case 2:
                    DEBUG_LOG("TrainController: setMotorSpeed called at: %lu", millis());
                    DEBUG_LOG("TrainController Demo: Motor forward (speed 35)");
                    duploHub.setMotorSpeed(35);
                    break;
                case 3:
                    DEBUG_LOG("TrainController Demo: Stop motor");
                    duploHub.stopMotor();
                    break;
                case 4:
                    DEBUG_LOG("TrainController Demo: Motor backward (speed -35)");
                    duploHub.setMotorSpeed(-35);
                    break;
                case 5:
                    DEBUG_LOG("TrainController Demo: Stop motor - demo complete");
                    duploHub.stopMotor();
                    duploHub.setLedColor(DuploEnums::DuploColor::GREEN);
                    break;
                case 6:
                    DEBUG_LOG("TrainController Demo: Playing sound - HORN");
                    duploHub.playSound(DuploEnums::DuploSound::HORN);
                    break;
                case 7:
                    DEBUG_LOG("TrainController Demo: Playing sound - BELL");
                    duploHub.playSound(DuploEnums::DuploSound::BRAKE);
                    demoStep = -1;
                    break;
            }
            demoStep++;
        }
    }
}
