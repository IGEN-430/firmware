/*
IGEN 430 Capstone Project
Ryan Lee
*/

#include "Wire.h"
#include "bhy.h"

//gpio pin definitions
#define SDA 21
#define SCL 22
#define INT 23
#define PWR 19


BHYSensor bhi160;

volatile bool intrToggled = false;

bool checkSensorStatus(void);

void bhyInterruptHandler(void){
    intrToggled = true;
}

void waitForBhyInterrupt(void){
    while (!intrToggled)
        ;
    intrToggled = false;
}

byte finderskeepers(void);

void setup(){

    pinMode(PWR,OUTPUT);
    digitalWrite(PWR,HIGH);
    delay(2500);
    
    Serial.begin(115200);
    if (Serial) Serial.println("Serial working");

    Wire.begin();
    int n = finderskeepers();
    if (n == -1) {
      Serial.println("No I2C connection found, Ending program");
      return;
    }

    delay(4000);

    attachInterrupt(INT,bhyInterruptHandler,RISING);
    Serial.println("attached interrupt");

    bhi160.begin(n); //BHY_I2C_ADDR
    Serial.println("beginning bhi160");

    if(!checkSensorStatus()) return;

    Serial.println("Sensor found over I2C! Product ID: 0x" + String(bhi160.productId, HEX));
    
    uint8_t revision = bhi160.getRevisionId();
    Serial.println(revision,HEX);
    
    //finished setup
    Serial.println("Finished setup");

    return;
}

void loop() {
  delay(1000);
}

/*find a slave device*/
byte finderskeepers() {
  byte error, address;
  Serial.print("I2C Scan\n");
  for (address = 0; address < 127; address++) {
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

bool checkSensorStatus(void)
{
    if (bhi160.status == BHY_OK)
        return true;

    if (bhi160.status < BHY_OK) /* All error codes are negative */
    {
        Serial.println("Error code: (" + String(bhi160.status) + "). " + bhi160.getErrorString(bhi160.status));

        return false; /* Something has gone wrong */
    }
    else /* All warning codes are positive */
    {
        Serial.println("Warning code: (" + String(bhi160.status) + ").");

        return true;
    }

    return true;
}
