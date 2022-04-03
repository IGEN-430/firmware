#include "Arduino.h"
#include "mpu_processing.h"

/* calculate accelerometer angles for tilt */
void Processor::accelAngles(double* ax, double* ay, double* az, double* aroll, double* apitch) {
    int temp;
    temp = *az * 90; //orientation of z axis
    if ((temp > 0 && (*ay * 90) > 0) || (temp < 0 && (*ay * 90) < 0)) { //case both positive or both negative
      *aroll = ((*ay * 90) * 95/100) + (temp * 5/100);
    }
    else { //if not goes with the xy axis
      *aroll = ((*ay * 90) * 95/100) + (-temp * 5/100);
    }
    if ((temp > 0 && (*ax * 90) > 0) || (temp < 0 && (*ax * 90) < 0)) { //case both positive or both negative
      *apitch = ((*ax * 90) * 95/100) + (temp * 5/100);
    }
    else { //if not goes with xy axis
      *apitch = ((*ax * 90) * 95/100) + (-temp * 5/100);
    }
    
    return;
}
/* get gyro integration for tilt */
void Processor::gyroInteg(double* gx, double* gy,double* groll,double* gpitch, double* grollp, double* gpitchp, char dt_num, char dt_denom) {
    *gx = (*gx * 5/8); // not sure why but factor of 3/4 fits data better to our measurements in testing phase 
    *gy = (*gy * 5/8); 

    *groll = *grollp + (*gx * dt_num/dt_denom);
    *gpitch = *gpitchp + (*gy * dt_num/dt_denom);
    //shift for next
    *grollp = *groll;
    *gpitchp = *gpitch;
    return;
}
/* Complementary filter to fuse data */
void Processor::complemFilter(double* groll, double* gpitch, double* aroll, double* apitch, double* croll, double* cpitch) {
    if( (*groll > 0 && *aroll > 0) || (*groll < 0 && *aroll < 0)) { //again making sure that signs are the same
    *croll = *groll * 0.05 + *aroll * 0.95;
    }
    else { //if not, goes with the accelerometer sign
      *croll = -*groll * 0.05 + *aroll * 0.95;
    }
    if ( (*gpitch > 0 && *apitch > 0) || (*gpitch < 0 && *apitch < 0)){
      *cpitch = *gpitch * gyro_num/dt_denom + *apitch * accel_num/dt_denom;
    }
    else {
      *cpitch = -*gpitch * gyro_num/dt_denom + *apitch * accel_num/dt_denom;
    }
    
    return;
}
