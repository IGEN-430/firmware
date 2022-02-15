/*
 * Ryan Lee 
 * IGEN 430 Capstone Project 
 * Arduino main code for ESP32 based project communicating with MPU6050 IMU over I2C
 */

//include
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include "Wire.h"
#include "mpu_cali.h"
#include <Preferences.h>
#include "mpu_processing.h"

//debug ifdef
#define DEBUG_
#define TEST_

//gpio pin definitions
//default I2C address 0x68
#define SDA 21
#define SCL 22
#define INT 23
#define PWR 19

#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

//function definitions
byte finderskeepers(void);
bool setup_calibration(void);
void getQuaternion(void);
void run_gen(void);

//class definitions
MPU6050 accelgyro;
Calibrator calibrator;
Preferences preferences;

//data initializers
int16_t global_offsets[N_DATA] = {0}; //accel x,y,z gyro x,y,z
int16_t global_offsets_last[N_DATA] = {0}; //last state saved

//quaternion holders
Quaternion q; //[w,x,y,z]

//orientation vectors
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

//raw values
int16_t ax, ay, az;
int16_t gx, gy, gz;

//mpu control/status vars
bool dmpReady = false;
int8_t interrupt_status;
int8_t device_status;
int16_t packetsize;
uint16_t fifocount;
uint8_t fifobuffer[64];
uint8_t bytes_toremove;

//-----interrupt detection routine-----
//-                                   -
//-------------------------------------
volatile bool mpu_interrupt = false;
void dmpDataReady() {
    mpu_interrupt = true;
}

void setup(){
    //initialize power to IMU
    pinMode(PWR,OUTPUT);
    digitalWrite(PWR,HIGH);
    delay(2500);
    
    //begin communication functions
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    preferences.begin("offset-values",false); //read-only false

    //determine if there is I2C connection
    int n = finderskeepers();
    if (n == -1) {
      Serial.println("No I2C connection found, Ending program");
      exit(0);
    }

    // initialize device
    Serial.println("Initializing I2C device...");
    accelgyro.initialize();

    //initalize interrupt pin
    pinMode(INT,INPUT);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    if (Serial) Serial.println("Serial working");
      
    //initalize the dmp -> necessary for interrupt flow rather than always polling
    device_status = accelgyro.dmpInitialize();
    Serial.println("Initializing DMP...");

    Serial.println("DMP Sample rate = "+String(accelgyro.getRate())); //auto: 4
    accelgyro.setRate(0x01);

    setup_calibration();
    
    if (device_status == 0) {
      accelgyro.setDMPEnabled(true);
    Serial.println("Enabling DMP...");

    attachInterrupt(digitalPinToInterrupt(INT), dmpDataReady, RISING);
    interrupt_status = accelgyro.getIntStatus();

    //set dmp ready flag
    dmpReady = true;
    Serial.println("DMP Ready...");

    packetsize = accelgyro.dmpGetFIFOPacketSize();
    Serial.println("DMP FIFO packet size is ..."+String(packetsize));
    
    }
    else 
      Serial.println("Unable to initalize DMP... Return code "+String(device_status));
      // error code 1 -- memory load failed
      // error code 2 -- dmp config updates failed
}

void loop() {
  delay(125);
  run_dmp();
}

/*setup calibration one time run when power on*/
bool setup_calibration() {
    byte holder;
    #ifndef TEST_
    //get offset values in preferences
    global_offsets[0] = preferences.getShort("accelx",0);
    global_offsets[1] = preferences.getShort("accely",0);
    global_offsets[2] = preferences.getShort("accelz",0);
    global_offsets[3] = preferences.getShort("gyrox",0);
    global_offsets[4] = preferences.getShort("gyroy",0);
    global_offsets[5] = preferences.getShort("gyroz",0);
    #endif

    for(int i=0;i<N_DATA;i++) {
        global_offsets_last[i] = global_offsets[i];
      }

    if (global_offsets[0] == 0 && global_offsets[3] == 0) { //multiple zero values unlikely
      Serial.println("Nothing found in Preferences...");
      calibrator.calibration(accelgyro,global_offsets,MAX_CAL_LOOPS,BUFF_SIZE); //reduced calibration 
    }
    else {
      Serial.println("Calibration starting with Preferences offset values...");
      calibrator.calibration(accelgyro,global_offsets,PREF_MAX_CAL_LOOPS,PREF_BUFF_SIZE); //one-time calibration
    }
    Serial.println("Writing offset values to Preferences NVM");
    //put offset values in preferences, only write to non-volatile mem if different by deadzone threshold (only 100,000 rewrites available)
    if (abs(global_offsets[0]-global_offsets_last[0]) > OFFSET_DEADZONE) preferences.putShort("accelx",global_offsets[0]);
    if (abs(global_offsets[1]-global_offsets_last[1]) > OFFSET_DEADZONE) preferences.putShort("accely",global_offsets[1]);
    if (abs(global_offsets[2]-global_offsets_last[2]) > OFFSET_DEADZONE) preferences.putShort("accelz",global_offsets[2]);
    if (abs(global_offsets[3]-global_offsets_last[3]) > OFFSET_DEADZONE) preferences.putShort("gyrox",global_offsets[3]);
    if (abs(global_offsets[4]-global_offsets_last[4]) > OFFSET_DEADZONE) preferences.putShort("gyroy",global_offsets[4]);
    if (abs(global_offsets[5]-global_offsets_last[5]) > OFFSET_DEADZONE) preferences.putShort("gyroz",global_offsets[5]);

    for(int i=0;i<N_DATA;i++) {
      global_offsets_last[i] = global_offsets[i];
    }
    Serial.println("Completed Calibration....");
}

/*find a slave device*/
byte finderskeepers() {
  byte error, address;
  Serial.print("I2C Scan\n");
  for (address = 100; address < 127; address++) {
    delay(500);//delay
    Serial.print("Checking ");
    Serial.println(address,HEX);
    
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C @ address 0x");
      Serial.println(address,HEX);
      Wire.endTransmission();
      return address;
    }
    else if (error == 4) {
      Serial.print("[ERROR] Uknown Error @ address 0x\n");
      Serial.println(address,HEX);
    }
  }
  Serial.println("No Connections Found\n");
  return (-1);
}

void run_gen(void) {
   accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
   Serial.print(ax); Serial.print("\t");
   Serial.print(ay); Serial.print("\t");
   Serial.print(az); Serial.print("\t");
   Serial.print(gx); Serial.print("\t");
   Serial.print(gy); Serial.print("\t");
   Serial.println(gz);
}


void run_dmp() {
    if (!dmpReady) return;
    
    //reset interrupt status
    mpu_interrupt = false;
    interrupt_status = accelgyro.getIntStatus();     

    //FIFO handling code
    fifocount = accelgyro.getFIFOCount();    
    
    if (accelgyro.dmpGetCurrentFIFOPacket(fifobuffer)) {
      accelgyro.dmpGetQuaternion(&q,fifobuffer);
      #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
       #endif
       #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            accelgyro.dmpGetAccel(&aa, fifobuffer);
            accelgyro.dmpGetGravity(&gravity, &q);
            accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
       #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            accelgyro.dmpGetAccel(&aa, fifobuffer);
            accelgyro.dmpGetGravity(&gravity, &q);
            accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif
      accelgyro.resetFIFO();
    }
    
    return;
}
