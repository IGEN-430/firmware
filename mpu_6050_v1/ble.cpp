#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "ble.h" //header file for final integration with main mpu6050 code 


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

//const String MyBLE::ANGLE_SERV = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
//const String MyBLE::ANGLE_CHAR = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

void MyBLE::initBLE() {
    BLEDevice::init("Motiv Sensor");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pBattery = pServer->createService(ANGLE_SERV);

    angleBLE = new BLECharacteristic(ANGLE_CHAR, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY); //add WRITE so app can send when user is in rest time
    pBattery->addCharacteristic(angleBLE);
    pBattery->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(ANGLE_SERV);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

void MyBLE::blePeripheralConnectHandler(BLEDevice central) {
    ;
}

void blePeripheralDisconnectHandler(BLEDevice central) {
    ;
} 

void MyBLE::setup() {
    initBLE();
}

bool MyBLE::bleComm(uint8_t dataval) {
    angleBLE->setValue(&dataval, 1);
    angleBLE->notify();  //pushes data reguarily (good for sensors)
    return 1;//everything okay
}
