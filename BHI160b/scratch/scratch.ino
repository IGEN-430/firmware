/* scratch file for i2c coding for ESP32 */

#include <Wire.h>
#include "bhy.h"
/*
GPIO PIN DEFS
*/
#define SDA 21 //GPIO pin 21
#define SCL 22 //GPIO pin 22
#define INT 23 //GPIO pin 23 interrupt
#define PWR 19 //GPIO pin 19
#define LED 18 //GPIO 18 -- for testing

/*
 * BHI160 REGISTER DEFS
 */
#define PROD_ID 0x91

/*
 * BHI160 READ ACCESS REGS
 */
#define PARAM_READ  0x3B //16 bits
#define P_PAGE_SEL  0x54 //8 bits -- bit 7 (0-read/1-write) --bit 6-0 parameter
#define PARAM_REQ   0x64 //
#define PARAM_ACK   0x3A //

#define I2C_ADDR 0x0

BHYSensor bhi160;

volatile bool intrToggled = false;

bool checkSensorStatus(void);

void bhyInterruptHandler(void)
{
    intrToggled = true;
}

void waitForBhyInterrupt(void)
{
    while (!intrToggled)
        ;
    intrToggled = false;
}

byte addr;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWR,OUTPUT); //set pwr to imu
  digitalWrite(PWR,HIGH); //set to high  
  pinMode(LED,OUTPUT); //set pwr to imu
  digitalWrite(LED,HIGH); //set to high   

  delay(3000);
  
  Serial.begin(115200);
  if (Serial){
    Serial.println("Serial working");
  }

  Wire.begin();
  addr = finderskeepers();

  bhi160.begin(addr);

  if(!checkSensorStatus()){
    Serial.println("[ERROR] Sensor Status");
    return;
  }
  else 
    Serial.println("All OK");

  attachInterrupt(INT,bhyInterruptHandler,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println(i2c_read(addr,1,PROD_ID),BIN);
  Serial.println("Looping");
  delay(3000);
}
/*find a slave device*/
byte finderskeepers() {
  byte error, address;
  Serial.print("I2C Scan\n");
  for (address = 0; address < 127; address++) {
    delay(500);//delay
    Serial.print("Checking ");
    Serial.println(address);
    
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
  return 0;
}
