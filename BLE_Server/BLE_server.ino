/*
    Based on Neil Kolban example
    lots changed by Harrison P for IGEN 430 Capstone Project
    Bluetooth ESP server 
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE_server.h" //header file for final integration with main mpu6050 code 
#include <ButtonEvent.h>
#include <ArduinoBLE.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BATTERY_UUID "0000180f-0000-1000-8000-00805f9b34fb" //standard battery service uuid -- causing a "Guru Meditation Error: Core 1 panic'ed"- with HEX addresses!
#define BATTERYCHAR_UUID "00002a19-0000-1000-8000-00805f9b34fb" //standard battery characteristic uuid
#define DELAY 500

  BLECharacteristic batteryLevel(BATTERYCHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY); //add WRITE so app can send when user is in rest time
  unsigned long lastExecutedMillis = 0; // variable to save the last executed time
  String DeviceName;

void initBLE() {
  
  BLEDevice::init("Motiv Sensor");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pBattery = pServer->createService(BATTERY_UUID);
  //BLEService *pService = pServer->createService(SERVICE_UUID);
  /*BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );*/

  pBattery->addCharacteristic(&batteryLevel);
  //batteryLevel.setValue(&batt,1);
  //pCharacteristic->setValue("Hello World says Neil");
  //pService->start();
  pBattery->start();
  //BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  //pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->addServiceUUID(BATTERY_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Bluetooth device active, waiting for connections...");

}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
} 

 void bleReConnect(){
  if (central) {
        BLE.stopScan();
        Serial.println("Found Device");
        if (central.connect()){
          Serial.println("Connected");
        } else {
          Serial.println("Trying to Connect");
          bleReConnect();
        }
      }
 }
 
void BLEReset(ButtonInformation* Sender) { //when BLE reset button is pressed
    Serial.println("BLE connection resetting");
    central.disconnect(); //returns true if disconnected
    BLE.scanForName(DeviceName);
    central = BLE.available();
    bleReConnect();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  initBLE();

   // register new connection handler
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  // register disconnect handler
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
 }

  DeviceName = //Get Device Name after connection established

  ButtonEvent.addButton(__, BLEReset);// set BLE button pin
   
  uint8_t batt = 99; 

void loop() {

  BLEDevice central = BLE.central();
  if (central)
  {
    while (central.connected()) {
          //int battery = analogRead(A0); //int batteryLevel = map(battery, 0, 1023, 0, 100); --> if smart battery implemented that can report charge level
          ;
    }
  }

  batteryLevel.setValue(&batt, 1);
  batteryLevel.notify();  //pushes data reguarily (good for sensors)

  ButtonEvent.loop();
  
  unsigned long currentMillis = millis();

  if (currentMillis - lastExecutedMillis >= DELAY) {
    lastExecutedMillis = currentMillis; // save the last executed time
      batt++;
      Serial.println(int(batt));
  }

  if (int(batt)==100)
  batt=0;

}
