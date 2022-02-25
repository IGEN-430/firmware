#include "Arduino.h"
#include "mpu_processing.h"

/* calculate accelerometer angles for tilt */
void Processor::accelAngles(int16_t &ax, int16_t &ay, float &aroll, float &apitch) {
    aroll = ay*90;
    apitch = ax*90;
    return;
}
/* get gyro integration for tilt */
void Processor::gyroInteg(int16_t &gx,int16_t &gy,float &groll,float &gpitch, float &grollp, float &gpitchp, float dt) {
    gx*90/PI; // why am i doing this? data fitting
    gy*90/PI; // why am i doing this? data fitting

    groll = grollp + (gx * dt)
    gpitch = gpitchp + (gy * dt)
    return;
}

void Processor::complemFilter(float &groll, float &gpitch, float &aroll, float &apitch, float &croll, float &cpitch) {
    croll = groll * 0.1 + aroll* 0.9
    cpitch = gpitch* 0.1 + apitch*0.9
    return;
}

void Processor::updateWindow(float* ptrRoll, float* ptrPitch, byte window, float &croll, float &cpitch) {
    for (byte i = 0; i < window; i--) {
        if (i < (window-1)) {
            *ptrRoll = *(ptrRoll+i)
            *ptrPitch = *(ptrPitch+i)
        }
        else {
            *ptrRoll = croll;
            *ptrPitch = cpitch;
        }
    }
    return;
}

void Processor::countReps(float *ptrRoll, float *ptrPitch) {
    
}