#include <Wire.h>

#include "bhy.h"
#include "bosch_pcb_7183_di03_bmi160-7183_di03-2-1-11824.h"

#define BHY_INT_PIN 23
#define PWR 19 //GPIO pin 19
#define LED 18

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

//global variables
byte addr;

void setup()
{
    pinMode(PWR,OUTPUT); //set pwr to imu
    digitalWrite(PWR,HIGH); //set to high  
    pinMode(LED,OUTPUT); //display LED
    digitalWrite(LED,HIGH); //set to high  
    
    delay(2000);
    
    Serial.begin(115200);
    Wire.begin();

    addr = finderskeepers();

    if (Serial)
    {
        Serial.println("Serial working");
    }

    attachInterrupt(BHY_INT_PIN, bhyInterruptHandler, RISING);

    bhi160.begin(BHY_I2C_ADDR2);

    // Check to see if something went wrong.
    if (!checkSensorStatus())
        return;

    Serial.println("Sensor found over I2C! Product ID: 0x" + String(bhi160.productId, HEX));

    Serial.println("Uploading Firmware.");
    bhi160.loadFirmware(bhy1_fw);

    if (!checkSensorStatus())
        return;

    intrToggled = false; /* Clear interrupt status received during firmware upload */
    waitForBhyInterrupt();  /* Wait for meta events from boot up */
    Serial.println("Firmware booted");

    /* Install a metaevent callback handler and a timestamp callback handler here if required before the first run */
    bhi160.run(); /* The first run processes all boot events */

    /* Link callbacks and configure desired virtual sensors here */

    if (checkSensorStatus())
        Serial.println("All ok");
}

void loop()
{
    if (intrToggled)
    {
        intrToggled = false;
        bhi160.run();
        checkSensorStatus();
    }
}

/*print errors for debugging*/
void errorhandler(String statement) {
  Serial.print("[ERROR]\t"+statement);
  return;
}
/*find a slave device*/
byte finderskeepers() {
  byte error, address;
  Serial.print("I2C Scan\n");
  for (address = 1; address < 127; address++) {
    
    delay(500);//delay
    Serial.print("Checking ");
    Serial.println(address);
    Serial.print("\r");
    
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C @ address 0x\n");
      Serial.println(address,HEX);
      return address;
    }
    else if (error == 4) {
      Serial.print("[ERROR]\tUknown Error @ address 0x\n");
      Serial.println(address,HEX);
    }
  }
  errorhandler("No Connections Found\n");
  return 0;
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
