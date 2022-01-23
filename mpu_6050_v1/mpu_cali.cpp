/*
* Ryan Lee
* IGEN 430 Capstone Project
* Calibration modules to run everytime the device is powered on
*/

#include <Arduino.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include "Wire.h"

bool Calibrator::calibration() {
    word loopcount = 0;
    byte ready = 0;
    long means[N_DATA]; //data structure [ax,ay,az,gx,gy,gz]
    
    long *p_means = calculate_mean(means);

    axo =- p_means[0]/acel_deadzone;
    ayo =- p_means[1]/acel_deadzone;
    azo =- p_means[2]/acel_deadzone;
    gxo =- p_means[3]/gyro_deadzone;
    gyo =- p_means[4]/gyro_deadzone;
    gzo =- p_means[5]/gyro_deadzone;

    while(1) {//no init offsets
        accelgyro.setXAccelOffset(axo);
        accelgyro.setYAccelOffset(ayo);
        accelgyro.setZAccelOffset(azo);
        accelgyro/setXGyroOffset(gxo);
        accelgyro/setYGyroOffset(gyo);
        accelgyro/setZGyroOffset(gzo);
        
        p_means = calculate_mean(means);

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

long* calculate_mean(long temp[N_DATA]) {
    int i = 0;
    while (i < num_meas_to_discard) { //while loop to skip through the measurements to remove
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        delay(2);
        i++;
    }
    i=0; //reset counter
    while (i < buffersize) { //summing the other measurements
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        temp[0]+=ax;
        temp[1]+=ay;
        temp[2]+=az;
        temp[3]+=gx;
        temp[4]+=gy;
        temp[5]+=gz;
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

