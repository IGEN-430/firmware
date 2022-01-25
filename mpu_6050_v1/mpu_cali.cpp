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

bool Calibrator::calibration(MPU6050 accelgyro) {
    word loopcount = 0;
    byte ready = 0;
    long means[N_DATA]; //data structure [ax,ay,az,gx,gy,gz]
    
    long *p_means = calculate_mean(means,accelgyro);

    axo =- p_means[0]/acel_deadzone;
    ayo =- p_means[1]/acel_deadzone;
    azo =- p_means[2]/acel_deadzone;
    gxo =- p_means[3]/gyro_deadzone;
    gyo =- p_means[4]/gyro_deadzone;
    gzo =- p_means[5]/gyro_deadzone;

    while(1) {//no init offsets
        ready = 0;
        accelgyro.setXAccelOffset(axo);
        accelgyro.setYAccelOffset(ayo);
        accelgyro.setZAccelOffset(azo);
        accelgyro.setXGyroOffset(gxo);
        accelgyro.setYGyroOffset(gyo);
        accelgyro.setZGyroOffset(gzo);
        
        p_means = calculate_mean(means,accelgyro);

        if (abs(p_means[0]) <= acel_deadzone) ready++;
        else axo = axo-p_means[0]/acel_deadzone;

        if (abs(p_means[1]) <= acel_deadzone) ready++;
        else ayo = ayo-p_means[1]/acel_deadzone;

        if (abs(p_means[2]) <= acel_deadzone) ready++;
        else azo = azo-p_means[2]/acel_deadzone;

        if (abs(p_means[3]) <= gyro_deadzone) ready++;
        else gxo = gxo-p_means[3]/gyro_deadzone;

        if (abs(p_means[4]) <= gyro_deadzone) ready++;
        else gyo = gyo-p_means[4]/gyro_deadzone;

        if (abs(p_means[5]) <= gyro_deadzone) ready++;
        else gzo = gzo-p_means[5]/gyro_deadzone;

        if ( ready == N_DATA) {
            Serial.println("[SUCCESS] Finished Calibration!");
            return true;
        }
        if (loopcount == MAX_CAL_LOOPS) {
            Serial.println("[FAIL] Calibration unable to return stable results after "+String(MAX_CAL_LOOPS)+"loops");
            return false;
        }
    }
}

long* Calibrator::calculate_mean(long temp[N_DATA],MPU6050 accelgyro) {
    int i = 0;
    int16_t xa, ya, za;
    int16_t xg, yg, zg;
    
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
        temp[i] = temp[i]/buffersize;
    }
    Serial.print("Results of measurements a/g:\t");
    Serial.print(temp[0]); Serial.print("\t");
    Serial.print(temp[1]); Serial.print("\t");
    Serial.print(temp[2]); Serial.print("\t\n");
    Serial.print(temp[3]); Serial.print("\t");
    Serial.print(temp[4]); Serial.print("\t");
    Serial.println(temp[5]);
}
