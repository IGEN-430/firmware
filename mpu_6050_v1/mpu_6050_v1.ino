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

//gpio pin definitions
#define SDA 21
#define SCL 22
#define INT 23
#define PWR 19

#define OUTPUT_READABLE_QUATERNION

//default I2C address 0x68

byte finderskeepers(void);

MPU6050 accelgyro;

Calibrator calibrator;

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
    
    Serial.begin(38400);
    Wire.begin();

    int n = finderskeepers();
    if (n == -1) {
      Serial.println("No I2C connection found, Ending program");
      exit(0);
    }

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    //initalize interrupt pin
    pinMode(INT,INPUT);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    if (Serial) Serial.println("Serial working");

    calibrator.calibration(accelgyro); //calibrate the IMU

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
    //testing();
    getQuaternion();
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

void testing() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

//    // blink LED to indicate activity
//    blinkState = !blinkState;
//    digitalWrite(LED_PIN, blinkState);
//    delay(100);
}
