/*
* Ryan Lee
* IGEN 430 Capstone Project
* Header file containing calibrator class and modules based on https://github.com/Protonerd/DIYino/blob/master/MPU6050_calibration.ino
*/
#include <Arduino.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include "Wire.h"

#define N_DATA 6
#define MAX_CAL_LOOPS 20

class Calibrator
{
    public:
        int axo,ayo,azo,gxo,gyo,gzo;
        bool calibration(MPU6050 accelgyro);
    private:
        //config
        int buffersize          = 1000; //readings to average for calibration
        int num_meas_to_discard = 100; //num inital measurements to discard
        int acel_deadzone       = 10; //accelerometer error allowed, lower value means higher precision
        int gyro_deadzone       = 10; //gyro error allowed, lwoer value means higher precision
        int acel_offset_div     = 8; //taken from src code
        int gyro_offset_div     = 4; //taken from src code
        //deadzone -- amount of variation between 2 consecutive measurements
        long* calculate_mean(long temp[N_DATA],MPU6050 accelgyro);
};
