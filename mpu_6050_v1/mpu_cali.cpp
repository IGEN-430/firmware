/*
* Ryan Lee
* IGEN 430 Capstone Project
* Calibration modules to run everytime the device is powered on
* based on https://github.com/Protonerd/DIYino/blob/master/MPU6050_calibration.ino
*/

#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include "Wire.h"
#include "mpu_cali.h"

bool Calibrator::calibration(MPU6050 accelgyro) {
    byte i = 0;
    byte ready = 0;
    int16_t means[N_DATA] = {0};

    //init offset values to 0
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

    Serial.println("Calculating mean for offset calibration");
    calculate_mean(accelgyro,means);

    while(i < MAX_CAL_LOOPS) {  
        ready = 0;
        axo -= means[0]/acel_deadzone;
        ayo -= means[1]/acel_deadzone;
        azo -= (16384-means[2])/acel_deadzone;
        gxo -= means[3]/gyro_deadzone;
        gyo -= means[4]/gyro_deadzone;
        gzo -= means[5]/gyro_deadzone;

        Serial.print("Results of measurements (outside function) a/g:\t");
        Serial.print(means[0]); Serial.print("\t");
        Serial.print(means[1]); Serial.print("\t");
        Serial.print(means[2]); Serial.print("\t");
        Serial.print(means[3]); Serial.print("\t");
        Serial.print(means[4]); Serial.print("\t");
        Serial.println(means[5]);
    
        accelgyro.setXAccelOffset(axo);
        accelgyro.setYAccelOffset(ayo);
        accelgyro.setZAccelOffset(azo);
        accelgyro.setXGyroOffset(gxo);
        accelgyro.setYGyroOffset(gyo);
        accelgyro.setZGyroOffset(gzo);

        Serial.print("offset values a/g:\t\t\t\t");
        Serial.print(axo); Serial.print("\t");
        Serial.print(ayo); Serial.print("\t");
        Serial.print(azo); Serial.print("\t");
        Serial.print(gxo); Serial.print("\t");
        Serial.print(gyo); Serial.print("\t");
        Serial.println(gzo);

        i++;

        Serial.println("Recalculating mean for offset calibration... loop #"+String(i));
        calculate_mean(accelgyro,means);

        if (abs(means[0]) <= acel_deadzone) ready++;
        else axo -= means[0]/acel_deadzone;

        if (abs(means[1]) <= acel_deadzone) ready++;
        else ayo -= means[1]/acel_deadzone;

        if (abs(means[2]) <= acel_deadzone) ready++;
        else azo -= (16384-means[2])/acel_deadzone;

        if (abs(means[3]) <= gyro_deadzone) ready++;
        else gxo -= means[3]/gyro_deadzone;

        if (abs(means[4]) <= gyro_deadzone) ready++;
        else gyo -= means[4]/gyro_deadzone;

        if (abs(means[5]) <= gyro_deadzone) ready++;
        else gzo -= means[5]/gyro_deadzone;

        if ( ready == N_DATA) {
            Serial.println("[SUCCESS] Finished Calibration!");

            Serial.print("Final offset values a/g:\t\t\t");
            Serial.print(axo); Serial.print("\t");
            Serial.print(ayo); Serial.print("\t");
            Serial.print(azo); Serial.print("\t");
            Serial.print(gxo); Serial.print("\t");
            Serial.print(gyo); Serial.print("\t");
            Serial.println(gzo);
            return true;
        }
        if (i >= MAX_CAL_LOOPS) {
            Serial.println("[FAIL] Calibration unable to return stable results after "+String(MAX_CAL_LOOPS)+"loops");
            
            Serial.print("Final offset values a/g:\t\t\t");
            Serial.print(axo); Serial.print("\t");
            Serial.print(ayo); Serial.print("\t");
            Serial.print(azo); Serial.print("\t");
            Serial.print(gxo); Serial.print("\t");
            Serial.print(gyo); Serial.print("\t");
            Serial.println(gzo);
            return false;
        }
    }
}

void Calibrator::calculate_mean(MPU6050 accelgyro,int16_t means[N_DATA]) {
    int i = 0;
    int16_t xa, ya, za;
    int16_t xg, yg, zg;
    int16_t temp[N_DATA] = {0};
    
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
    Serial.print("Results of measurements (inside function) a/g:\t");
    Serial.print(temp[0]); Serial.print("\t");
    Serial.print(temp[1]); Serial.print("\t");
    Serial.print(temp[2]); Serial.print("\t");
    Serial.print(temp[3]); Serial.print("\t");
    Serial.print(temp[4]); Serial.print("\t");
    Serial.println(temp[5]);
}
