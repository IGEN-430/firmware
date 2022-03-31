  /*
 * Ryan Lee 
 * IGEN 430 Capstone Project 
 * Arduino main code for ESP32 based project communicating with MPU6050 IMU over I2C
 */

//include
// #include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include "MPU6050.h"
#include "Wire.h"
#include "mpu_cali.h"
#include "mpu_processing.h"
#include "ble.h"

//debug ifdef
#define DEBUG_

//gpio pin definitions
//default I2C address 0x68
#define PWR 33 //tinypico is 33, dev module is 19

#define NCOUNT 3
#define G 8192 
// 4g -> 8192
// 8g -> 4096
#define A 0x01
//0x01 is 4g
//0x02 is 8g
#define MS 9.8 // cm/s^2 per g
#define GYRO_G 131 // this is +/-250 deg/s - therefore divide by this to get deg/s 
#define R 0.92

//function definitions
byte finderskeepers(void);
bool calibrate(void);
void getQuaternion(void);
void get_angles(void);
bool checkCalStatus(void);
void outputAngles(void);


//class definitions
MPU6050 accelgyro;
Calibrator calibrator;
Processor p;
MyBLE ble;

//data initializers
int16_t global_offsets[N_DATA] = {704,-1286,-134,36,71,22}; //accel x,y,z gyro x,y,z
int16_t global_offsets_last[N_DATA] = {704,-1286,-134,36,71,22}; //last state saved

//raw values
int16_t ax, ay, az, gx, gy, gz;
double ax_s, ay_s, az_s, gx_s, gy_s, gz_s; // summed values
double gx_in=0,gx_inL=0, gx_out=0,gx_outL=0,gy_in=0,gy_inL=0,gy_out=0,gy_outL=0;
double aroll, apitch, groll, gpitch, croll, cpitch; // current calculated roll/pitch values
double grollp, gpitchp;  // the last roll pitch values for integration

uint8_t output;

void setup(){
    //initialize power to IMU
    pinMode(PWR,OUTPUT);
    digitalWrite(PWR,HIGH);
    delay(2500);
    
    //begin communication functions
    Serial.begin(38400);
    Wire.begin();

    //determine if there is I2C connection
    int n = finderskeepers();
    if (n == -1) {
      #ifdef DEBUG_
      Serial.println("No I2C connection found, Ending program");
      #endif
      exit(0);
    }
    
//    //start ble
    ble.setup();

    // initialize device
    #ifdef DEBUG_
    Serial.println("Initializing I2C device...");
    #endif
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    if (Serial) Serial.println("Serial working");

    //device configuratino
    accelgyro.setFullScaleAccelRange(A); //this is +/- 8g therefore to get accel in g's divide # by 4096
    accelgyro.setDLPFMode(0x04);
    #ifdef DEBUG_
    Serial.println("DLPF mode = "+String(accelgyro.getDLPFMode()));
    #endif
    
    calibrate();
}

void loop() {
  get_angles(); //update angles
  output = (int8_t) cpitch;
  ble.bleComm(cpitch); //cast cpitch to uint8_t to send over ble
  
//  sendESPnow();
  
  outputAngles(); //print output angles
//  checkCalStatus(); //poll if angle greater than 180 -> recalibrate
  
  delay(50);
}


/*function to check from ble whether it is time to recalibrate or not */
bool checkCalStatus(){
  if (gpitch > 180 || gpitch < -180  || groll > 180 || groll < -180  ) {
    Serial.println("Recalibrating");  
    groll=0;gpitch=0;
    calibrate();
  }
}

/*setup calibration one time run when power on*/
bool calibrate() {
    byte holder;

    for(int i=0;i<N_DATA;i++) {
        global_offsets_last[i] = global_offsets[i];
      }

    if (global_offsets[0] == 0 && global_offsets[3] == 0) { //multiple zero values unlikely
      calibrator.calibration(accelgyro,global_offsets,MAX_CAL_LOOPS,BUFF_SIZE); //reduced calibration 
    }
    else {
      calibrator.calibration(accelgyro,global_offsets,PREF_MAX_CAL_LOOPS,PREF_BUFF_SIZE); //one-time calibration
    }
    for(int i=0;i<N_DATA;i++) {
      global_offsets_last[i] = global_offsets[i];
    }
}

/*find a slave device*/
byte finderskeepers() {
  byte error, address;
  Serial.print("I2C Scan\n");
  for (address = 100; address < 127; address++) {
    delay(500);//delay
    
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      #ifdef DEBUG_
      Serial.print("I2C @ address 0x");
      Serial.println(address,HEX);
      #endif
      Wire.endTransmission();
      return address;
    }
    else if (error == 4) {
      break;
    }
  }
  return (-1);
}

void get_angles(void) { //probably should switch to array
  uint8_t i=0;
  ax_s = 0; ay_s = 0; az_s = 0;
  gx_s = 0; gy_s = 0; gz_s = 0;
  
  do { //take measurements for average
    accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
     ax_s += ax;
     ay_s += ay;
     az_s += az;
     gx_s += gx;
     gy_s += gy;
     gz_s += gz;
     i++;
  } while (i < NCOUNT);
  ax_s = ax_s/NCOUNT;
  ay_s = ay_s/NCOUNT;
  az_s = az_s/NCOUNT;
  gx_s = gx_s/NCOUNT;
  gy_s = gy_s/NCOUNT;
  gz_s = gz_s/NCOUNT;

  //divid values by g to get values with unit g
  ax_s = ax_s/G; //after ax/G -> units = g then x 9.8m/s^2 per g
  ay_s = ay_s/G;
  az_s = az_s/G;

  //divide values to get deg/s
  gx_s = gx_s/GYRO_G;
  gy_s = gy_s/GYRO_G;
  gz_s = gz_s/GYRO_G;

//  //update lasts
//  gx_inL = gx_s;
//  gy_inL = gy_s;
//
//  //update inputs values
//  gx_s = gx_in;
//  gy_s = gy_in;
//
//  //update lasts
//  gx_outL = gx_out;
//  gy_outL = gy_out;
//
//  gx_out = gx_in - gx_inL + (gx_outL * R);
//  gy_out = gy_in - gy_inL + (gy_outL * R);


  p.accelAngles(&ax_s,&ay_s,&az_s,&aroll,&apitch);
  p.gyroInteg(&gx_s,&gy_s,&groll,&gpitch,&grollp,&gpitchp,p.dt_num,p.dt_denom);
  p.complemFilter(&groll,&gpitch,&aroll,&apitch,&croll,&cpitch);
}

void outputAngles(void)
{
  Serial.print(aroll); Serial.print(",");
  Serial.println(apitch);
  Serial.print(groll); Serial.print(",");
  Serial.println(gpitch);
  Serial.print("Comp Filter:  "); Serial.print(croll);Serial.print(",");Serial.println(cpitch);
}
