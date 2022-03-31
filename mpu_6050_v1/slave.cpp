//#include <esp_now.h>
//#include <WiFi.h>
//#include <Arduino.h>
//#include "slave.h"
//
///*
//     esp32now slave 
//*/
//
////#define USE_FIXED_CHANNEL   //Uncomment for no channel search
//
//void Slave::WiFiReset() {
//  WiFi.persistent(false);
//  WiFi.disconnect();
//  WiFi.mode(WIFI_OFF);
//  delay(100);
//}
//
//
//void Slave::setup()
//{
//  Serial.begin(115200);
//  Serial.print("\r\n\r\n");
// 
//#ifdef USE_FIXED_CHANNEL
//  uint8_t primaryChan = FIXED_CHANNEL;
//#else
//  for (uint8_t primaryChan = LOWEST_CHANNEL; primaryChan <= HIGHEST_CHANNEL; primaryChan++) {
//#endif
//  WiFiReset();
//  WiFi.mode(WIFI_STA);
//  
//  delay(100);
//  
// esp_wifi_set_mac(ESP_IF_WIFI_STA, &CustomMac[0]);
// 
//  delay(250);
// 
//  Serial.print("Channel no: ");
//  Serial.println(primaryChan);
//  Serial.println( WiFi.macAddress() );
//  
//  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
//  wifi_second_chan_t secondChan = WIFI_SECOND_CHAN_NONE;
//  ESP_ERROR_CHECK(esp_wifi_set_channel(primaryChan, secondChan));
//  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
// 
//  WiFi.printDiag(Serial);
// 
//  WiFi.disconnect();
//
//  Serial.print("Wifi Channel: "); Serial.println(WiFi.channel());
//
//  if (esp_now_init() == ESP_OK)
//  {
//    Serial.println("ESP NOW INIT!");
//  }
//  else
//  {
//    Serial.println("ESP NOW INIT FAILED....");
//  }
//
//  memcpy( &master.peer_addr, masterDeviceMac, 6 );
//  master.channel = primaryChan;
//  master.encrypt = 0;
//  master.ifidx = ESP_IF_WIFI_STA;
//
//#ifdef USE_FIXED_CHANNEL
//  if ( esp_now_add_peer(masterNode) == ESP_OK)
//  {
//    Serial.println("Added Peer!");
//  }
//#else
//  //Add node irst time, else  replace
//  if (primaryChan == LOWEST_CHANNEL) {
//    if ( esp_now_add_peer(masterNode) == ESP_OK)
//    {
//      Serial.println("Added Peer!");
//    }
//  } else {
//    if (esp_now_mod_peer(masterNode) == ESP_OK)
//
//    {
//      Serial.println("Modified Peer!");
//    }
//  }
//#endif
//
//  esp_now_register_send_cb(OnDataSent);
//  esp_now_register_recv_cb(OnDataRecv);
//}
//
//void Slave::loop()
//{
//  
//  esp_err_t sendResult = esp_now_send(master.peer_addr, &myData, sizeof(myData)); //maxDataFrameSize);
//  if (sendResult == ESP_OK) {
//    Serial.println("Success");
//  } else if (sendResult == ESP_ERR_ESPNOW_NOT_INIT) {
//    // How did we get so far!!
//    Serial.println("ESPNOW not Init.");
//  } else if (sendResult == ESP_ERR_ESPNOW_ARG) {
//    Serial.println("Invalid Argument");
//  } else if (sendResult == ESP_ERR_ESPNOW_INTERNAL) {
//    Serial.println("Internal Error");
//  } else if (sendResult == ESP_ERR_ESPNOW_NO_MEM) {
//    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
//  } else if (sendResult == ESP_ERR_ESPNOW_NOT_FOUND) {
//    Serial.println("Peer not found.");
//  } 
//  else if (sendResult == ESP_ERR_ESPNOW_IF) {
//    Serial.println("Interface Error.");
//  }   else {
//    Serial.printf("\r\nNot sure what happened\t%d", sendResult);
//  }
//}
//
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
//{
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//  dataSent = (status == ESP_NOW_SEND_SUCCESS ? 1 : -1);
//}
//
//void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
//{
//  //Serial.printf("\r\nReceived\t%d Bytes\t%d\n", data_len, data[0]);
//  Serial.printf("\r\nReceived\t%d Bytes\t%s\n", data_len, data);
//}
