/*
 * Ryan Lee 
 * IGEN 430 Capstone Project 
 * MPU6050 & ESP32 6 DOF IMU
 */

//include
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include "Wire.h"
#include "mpu_cali.h"
#include <EEPROMex.h> //write long-term to EEPROM

//debug ifdef
#define DEBUG_

//gpio pin definitions
//default I2C address 0x68
#define SDA 21
#define SCL 22
#define INT 23
#define PWR 19

#define OUTPUT_READABLE_QUATERNION

//EEPROM definitions
#define EEPROM_OFFSET_LENGTH 2 //2 byte length (16 bit, int value)
#define EEPROM_START 20 //start address
#define EEPROM_SIZE 512 // ESP32 512B EEPROM

//function definitions
byte finderskeepers(void);

//class definitions
MPU6050 accelgyro;
Calibrator calibrator;

//data initializers
int16_t global_offsets[N_DATA] = {0}; //accel x,y,z gyro x,y,z
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

int16_t eeprom_read;
uint16_t eeprom_update;

//orientation vars
Quaternion q; //[w,x,y,z]

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
    Serial.begin(38400);
    Wire.begin();
    EEPROM.begin(EEPROM_OFFSET_LENGTH);

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

    setup_calibration();
      
    //initalize the dmp -> necessary for interrupt flow rather than always polling
    device_status = accelgyro.dmpInitialize();
    Serial.println("Initializing DMP...");

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
    // read raw accel/gyro measurements from device
     delay(2500);
    getQuaternion();
}
bool setup_calibration() {
    //IMU calibration based on EEPROM
    EEPROM.setMemPool(EEPROM_START,EEPROM_SIZE); //set to 512B so if written over, return error
    #ifdef DEBUG_
      EEPROM.setMaxAllowedWrites(60);
    #endif
    if (EEPROM.isReady()) {
      eeprom_read = (int16_t) EEPROM.readInt(EEPROM_START); //read to see if written to EEPROM
    }

    if (eeprom_read == 0) {
      Serial.println("Nothing found in EEPROM...");
      calibrator.calibration(accelgyro,global_offsets,EEPROM_MAX_CAL_LOOPS,EEPROM_BUFF_SIZE); //reduced calibration 
    }
    else {
      Serial.println("Calibration starting with EEPROM offset values...");
      int8_t n = EEPROM_START;
      for(byte i = 0;i<N_DATA;i++) {
        global_offsets[i] = EEPROM.readInt(n);
        n += EEPROM_SIZE;
      }
      calibrator.calibration(accelgyro,global_offsets,MAX_CAL_LOOPS,BUFF_SIZE); //one-time calibration
      n = EEPROM_START;
      for (byte i = 0;i<N_DATA;i++) {
        eeprom_update = (uint16_t) global_offsets[i];
        EEPROM.updateInt(n,eeprom_update);
        n += EEPROM_SIZE;
      }
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

void getQuaternion() {
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
       accelgyro.resetFIFO();
    }
    return;
}