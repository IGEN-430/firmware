#include "Arduino.h"
#include "mpu_processing.h"

/* calculate accelerometer angles for tilt */
void Processor::accelAngles(double* ax, double* ay, double* az, double* aroll, double* apitch) {
    double temp;
    temp = *az * 90;
    if ((temp > 0 && (*ay * 90) > 0) || (temp < 0 && (*ay * 90) < 0)) {
      *aroll = ((*ay * 90) * 0.7) + (temp * 0.3);
    }
    else {
      *aroll = ((*ay * 90) * 0.7) + (-temp * 0.3);
    }
    if ((temp > 0 && (*ax * 90) > 0) || (temp < 0 && (*ax * 90) < 0)) {
      *apitch = ((*ax * 90.0) * 0.7) + (temp * 0.3);
    }
    else {
      *apitch = ((*ax * 90.0) * 0.7) + (-temp * 0.3);
    }
    
    return;
}
/* get gyro integration for tilt */
void Processor::gyroInteg(double* gx, double* gy,double* groll,double* gpitch, double* grollp, double* gpitchp, float dt) {
    *gx = (*gx * 3/4); // why am i doing this? data fitting
    *gy = (*gy * 3/4); // why am i doing this? data fitting

    *groll = *grollp + (*gx * dt);
    *gpitch = *gpitchp + (*gy * dt);
    //shift for next
    *grollp = *groll;
    *gpitchp = *gpitch;
    return;
}
/* Complementary filter to fuse data */
void Processor::complemFilter(double* groll, double* gpitch, double* aroll, double* apitch, double* croll, double* cpitch) {
    *croll = *groll * 0.1 + *aroll * 0.9;
    *cpitch = *gpitch * 0.1 + *apitch * 0.9;
    return;
}
