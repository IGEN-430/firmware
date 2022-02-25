#include "Arduino.h"

/* calculate accelerometer angles for tilt */
void accelAngles(int16_t &ax, int16_t &ay, float &aroll, float &apitch) {
    aroll = ay*90;
    apitch = ax*90;
}
/* get gyro integration for tilt */
void gyroInteg(int16_t &gx,int16_t &gy,float &groll,float &gpitch, float &grollp, float &gpitchp, float dt) {
    gx*90/3.14159;
    gy*90/3.14159;

    groll = grollp + (gx * dt)
    gpitch = gpitchp + (gy * dt)
}

void complemFilter(float &groll, float &gpitch, float &aroll, float &apitch, float &croll, float & cpitch) {
    croll = groll * 0.1 + aroll* 0.9
    cpitch = gpitch* 0.1 + apitch*0.9
}