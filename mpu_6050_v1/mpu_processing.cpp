#include "Arduino.h"
#include "mpu_processing.h"

/* calculate accelerometer angles for tilt */
void Processor::accelAngles(double* ax, double* ay, double* az, double* aroll, double* apitch) {
    double temp;
    temp = *az * 90; //orientation of z axis
    if ((temp > 0 && (*ay * 90) > 0) || (temp < 0 && (*ay * 90) < 0)) { //case both positive or both negative
      *aroll = ((*ay * 90) * 0.7) + (temp * 0.3);
    }
    else { //if not goes with the xy axis
      *aroll = ((*ay * 90) * 0.7) + (-temp * 0.3);
    }
    if ((temp > 0 && (*ax * 90) > 0) || (temp < 0 && (*ax * 90) < 0)) { //case both positive or both negative
      *apitch = ((*ax * 90.0) * 0.7) + (temp * 0.3);
    }
    else { //if not goes with xy axis
      *apitch = ((*ax * 90.0) * 0.7) + (-temp * 0.3);
    }
    
    return;
}
/* get gyro integration for tilt */
void Processor::gyroInteg(double* gx, double* gy,double* groll,double* gpitch, double* grollp, double* gpitchp, float dt) {
    *gx = (*gx * 3/4); // not sure why but factor of 3/4 fits data better to our measurements in testing phase 
    *gy = (*gy * 3/4); 

    *groll = *grollp + (*gx * dt);
    *gpitch = *gpitchp + (*gy * dt);
    //shift for next
    *grollp = *groll;
    *gpitchp = *gpitch;
    return;
}
/* Complementary filter to fuse data */
void Processor::complemFilter(double* groll, double* gpitch, double* aroll, double* apitch, double* croll, double* cpitch) {
    if( (*groll > 0 && *aroll > 0) || (*groll < 0 && *aroll < 0)) { //again making sure that signs are the same
    *croll = *groll * 0.2 + *aroll * 0.8;
    }
    else { //if not, goes with the accelerometer sign
      *croll = -*groll * 0.2 + *aroll * 0.8;
    }
    if ( (*gpitch > 0 && *apitch > 0) || (*gpitch < 0 && *apitch < 0)){
      *cpitch = *gpitch * 0.1 + *apitch * 0.9;
    }
    else {
      *cpitch = -*gpitch * 0.1 + *apitch * 0.9;
    }
    
    return;
}
