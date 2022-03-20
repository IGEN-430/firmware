#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

//#define ANGLE_SERV  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
//#define ANGLE_CHAR   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
//#define BATTERY_UUID "0000180f-0000-1000-8000-00805f9b34fb" //standard battery service uuid -- causing a "Guru Meditation Error: Core 1 panic'ed"- with HEX addresses!
//#define BATTERYCHAR_UUID "00002a19-0000-1000-8000-00805f9b34fb" //standard battery characteristic uuid
#define DELAY 500

class MyBLE
{
    public:
        unsigned long lastExecutedMillis = 0; // variable to save the last executed time
        void setup(void);
        bool bleComm(uint8_t dataval);
    private:
        void initBLE(void);
        void blePeripheralConnectHandler(BLEDevice central);
        void blePeripheralDisconnectHandler(BLEDevice central);
        static constexpr const char* ANGLE_SERV = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
        static constexpr const char* ANGLE_CHAR = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
        BLECharacteristic* angleBLE;
};
