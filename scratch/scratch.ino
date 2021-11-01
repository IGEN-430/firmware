/* scratch file for SPI coding for ESP32 */

#include <Wire.h>

#define SDA 21 //GPIO pin 21
#define SCL 22 //GPIO pin 22
#define INT 23 //An input

/* i2c write to bhi160b example
 * slave addr - 7 bits
 * r/w - 1 bit (0 for write)
 * acknowledgement - 1 bit (0)
 * register address - 8 bits
 * acknowledgement - 1 bit (0)
 * data to reg (n) - 8 bits
 * acknowledgement - 1 bit (0)
 * data to reg (n+1) - 8 bits
 * acknowledgement - 1 bit (0)
 */

byte i2c_rx; //master data received from I2C bus

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
/*============================================ FUNCTIONS ==================================================*/
/*print errors for debugging*/
void errorhandler(String statement) {
  Serial.print("[ERROR]\t"+statement);
  return;
}
/*find a slave device*/
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
      Serial.print("[ERROR]\tUknown Error @ address 0x");
      Serial.println(address,HEX);
    }
  }
  errorhandler("No Connections Found");
  return 0;
}
/*read from i2c slave as master*/
byte i2c_read(byte addr,word nbytes){
  Wire.requestFrom(addr,nbytes); //request nbytes from address
  if (Wire.available()) {
    return Wire.read();
  }
  else {
    errorhandler("Address Unavailable");
  }
  return 0;
}
/*write to i2c slave as master*/
void i2c_write(byte data[], int length){
  Wire.write(data,length);
  return;
}
