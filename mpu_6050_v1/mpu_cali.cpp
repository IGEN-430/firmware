/*
* Ryan Lee
* IGEN 430 Capstone Project
* Calibration modules to run everytime the device is powered on
* based on https://github.com/Protonerd/DIYino/blob/master/MPU6050_calibration.ino
*/

#include <Arduino.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include "Wire.h"
#include "mpu_cali.h"

#define DEBUG_

/*
* calibration function basically takes a sample of measurements and tries to adjust offsets
* until the deadzone is reached (deadzone specified in .h file)
*/
bool Calibrator::calibration(MPU6050 accelgyro, int16_t offsets[N_DATA],byte max_loops, int buffersize) {
    byte i = 0;
    byte ready = 0;
    int16_t means[N_DATA] = {0};

    //init offset values to global offset values
    accelgyro.setXAccelOffset(offsets[0]);
    accelgyro.setYAccelOffset(offsets[1]);
    accelgyro.setZAccelOffset(offsets[2]);
    accelgyro.setXGyroOffset(offsets[3]);
    accelgyro.setYGyroOffset(offsets[4]);
    accelgyro.setZGyroOffset(offsets[5]);

    calculate_mean(accelgyro,means,buffersize);

    while(i < max_loops) {  
        ready = 0;
        axo -= means[0]/acel_offset_div; //divisor is like a dampening factor
        ayo -= means[1]/acel_offset_div;
        if (i == 0)
          azo -= (16384-means[2])/acel_offset_div;
        else 
          azo -= means[2]/acel_offset_div;
        gxo -= means[3]/gyro_deadzone;
        gyo -= means[4]/gyro_deadzone;
        gzo -= means[5]/gyro_deadzone;
    
        accelgyro.setXAccelOffset(axo);
        accelgyro.setYAccelOffset(ayo);
        accelgyro.setZAccelOffset(azo);
        accelgyro.setXGyroOffset(gxo);
        accelgyro.setYGyroOffset(gyo);
        accelgyro.setZGyroOffset(gzo);

        i++;
        calculate_mean(accelgyro,means,buffersize);

        if (abs(means[0]) <= acel_deadzone) ready++;
//        else axo -= means[0]/acel_deadzone;
        if (abs(means[1]) <= acel_deadzone) ready++;
//        else ayo -= means[1]/acel_deadzone;
        if (abs(means[2]) <= acel_deadzone) ready++;
//        else azo -= (16384-means[2])/acel_deadzone;
        if (abs(means[3]) <= gyro_deadzone) ready++;
//        else gxo -= means[3]/gyro_deadzone;
        if (abs(means[4]) <= gyro_deadzone) ready++;
//        else gyo -= means[4]/gyro_deadzone;
        if (abs(means[5]) <= gyro_deadzone) ready++;
//        else gzo -= means[5]/gyro_deadzone;
        if (ready == N_DATA) {
            return true;
        }
        if (i >= max_loops) {
            return false;
        }
    }
}
/*
 * grab sample data and calculate the average -- discards num_meas_to_discard measurements first
 */
void Calibrator::calculate_mean(MPU6050 accelgyro,int16_t means[N_DATA],int buffersize) {
    int i = 0;
    int16_t xa, ya, za;
    int16_t xg, yg, zg;
    long temp[N_DATA] = {0};
    
    while (i < num_meas_to_discard) { //while loop to skip through the measurements to remove
        accelgyro.getMotion6(&xa, &ya, &za, &xg, &yg, &zg);
        delay(2);
        i++;
    }
    i=0; //reset counter
    while (i < buffersize) { //summing the other measurements
        accelgyro.getMotion6(&xa, &ya, &za, &xg, &yg, &zg);
        temp[0]+=xa;
        temp[1]+=ya;
        temp[2]+=za;
        temp[3]+=xg;
        temp[4]+=yg;
        temp[5]+=zg;
        delay(2);
        i++;
    }
    for (i=0;i<N_DATA;i++) { //calculate mean
        means[i] = temp[i]/buffersize;
    }
//    Serial.print("Results of measurements (inside function) a/g:\t");
//    Serial.print(temp[0]); Serial.print("\t");
//    Serial.print(temp[1]); Serial.print("\t");
//    Serial.print(temp[2]); Serial.print("\t");
//    Serial.print(temp[3]); Serial.print("\t");
//    Serial.print(temp[4]); Serial.print("\t");
//    Serial.println(temp[5]);
}
