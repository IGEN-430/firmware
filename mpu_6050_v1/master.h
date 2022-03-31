//#include <esp_now.h>
//#include <WiFi.h>
//#include <Arduino.h>
//
//
//#define WIFI_CHANNEL 10
//#define FIXED_CHANNEL 6
//
//class ESPMaster
//{
//    private:
//        //Use custom MAC for both master and slave
//        uint8_t CustomMac[] = {0xB4, 0xE6, 0x2D, 0xE9, 0xFE, 0x6E}; 		
//        uint8_t slaveDeviceMac[] = {0x3C, 0x71, 0xBF, 0x03, 0x3D, 0x30};  
//        const byte maxDataFrameSize = 200;
//        uint8_t dataToSend[maxDataFrameSize];
//        byte cnt = 0;
//        int dataSent = 0;
//
//        esp_now_peer_info_t slave;
//        const esp_now_peer_info_t *peer = &slave;
//        void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
//        void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
//    public:
//        void WiFiReset();
//        void setup();
//        void prntmac(const uint8_t *mac_addr);
//        void loop();
//}
