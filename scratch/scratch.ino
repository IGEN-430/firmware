/* scratch file for SPI coding for ESP32 */

#include <Wire.h>

#define SDA 21 //GPIO pin 21
#define SCL 22 //GPIO pin 22
#define INT 23 //Any GPIO pin

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  byte addr = finderskeepers();
  }

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Hello Hi\n");
  delay(2000);
}

byte finderskeepers() {
  byte error, address;
  Serial.print("I2C Scan");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C @ address 0x");
      Serial.println(address,HEX);
      return address;
    }
    else if (error == 4) {
      Serial.print("Uknown Error @ address 0x");
      Serial.println(address,HEX);
    }
  }
}
