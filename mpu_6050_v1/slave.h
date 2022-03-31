//#include <esp_now.h>
//#include <WiFi.h>
//#include <Arduino.h>
//
//#define HIGHEST_CHANNEL  11
//#define LOWEST_CHANNEL  1
//#define FIXED_CHANNEL   6
//
///* SLAVE SENDS DATA !!!!! */
//class Slave
//{
//    public:
//        typedef struct msg {
//            int8_t roll,pitch;
//        } msg;
//        msg myData;
//        void WiFiReset();
//        void setup();
//        void loop();
//        
//    private:
//        //Use custom MAC for both master and slave
//        uint8_t masterDeviceMac[] = {0xB4, 0xE6, 0x2D, 0xE9, 0xFE, 0x6E};  
//        uint8_t CustomMac[] = {0x3C, 0x71, 0xBF, 0x03, 0x3D, 0x30};
//        esp_now_peer_info_t master;
//        const esp_now_peer_info_t *masterNode = &master;
//        esp_err_t sendResult;
//        // const uint8_t maxDataFrameSize = 200;
//        uint8_t dataToSend[maxDataFrameSize];
//        // byte cnt = 0;
//        // int dataSent = 0;
//        int wifi_channel = 1;
//        int ms_sleep = 0;
//        void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
//        void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
//}
