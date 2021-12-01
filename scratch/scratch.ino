/* scratch file for SPI coding for ESP32 */

#include <Wire.h>
//#include <bosch_pcb_7183_di03_bmi160-7183_di03-2-1-11824.h>

#define SDA 21 //GPIO pin 21
#define SCL 22 //GPIO pin 22
#define INT 23 //GPIO pin 23 interrupt
#define PWR 19 //GPIO pin 19
#define LED 18

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
byte addr;
String buffer;
int i;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWR,OUTPUT); //set pwr to imu
  digitalWrite(PWR,HIGH); //set to high  
  pinMode(LED,OUTPUT); //set pwr to imu
  digitalWrite(LED,HIGH); //set to high  
  
  delay(20000);
  
  
  Wire.begin();
  Serial.begin(115200);
  
  }

void loop() {
  // put your main code here, to run repeatedly:
  if (addr == NULL){
    addr = finderskeepers();
  }
  i++;
  Serial.print("Loop");
  Serial.println(i);
  Serial.print('\r');
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
  Serial.print("I2C Scan\n");
  for (address = 0; address < 127; address++) {
    
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
