//#include <esp_now.h>
//#include <WiFi.h>
//#include <Arduino.h>
//#include "master.h"
//
///*
//     espnow master 
//*/
//
//void WiFiReset() {
//  WiFi.persistent(false);
//  WiFi.disconnect();
//  WiFi.mode(WIFI_OFF);
//}
//
//
//void setup()
//{
//  Serial.begin(115200);
//  Serial.print("\r\n\r\n");
//  WiFiReset();
//
//  WiFi.mode(WIFI_AP_STA );
//
//  esp_wifi_set_mac(ESP_IF_WIFI_STA, &CustomMac[0]);
//  
//  Serial.print("Connecting to ");
//  Serial.println( WiFi.softAPmacAddress() );
//
//  Serial.println( WiFi.macAddress() );
//
//  //Force espnow to use a specific channel
//  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
//  
//  uint8_t primaryChan = FIXED_CHANNEL;
//  wifi_second_chan_t secondChan = WIFI_SECOND_CHAN_NONE;
//  esp_wifi_set_channel(primaryChan, secondChan);
//
//  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
//
//  WiFi.disconnect();
//
//  Serial.print("Wifi Channel: "); Serial.println(WiFi.channel());
//
//  if (esp_now_init() == ESP_OK)
//  {
//    Serial.println("ESPNow Init Success!");
//  }
//  else
//  {
//    Serial.println("ESPNow Init Failed....");
//  }
//
//  //Add the slave node to this master node
//  memcpy( &slave.peer_addr, &slaveDeviceMac[0], 6 );
//  slave.channel = FIXED_CHANNEL;
//  slave.encrypt = 0;
//  slave.ifidx = ESP_IF_WIFI_STA;
//
//  if ( esp_now_add_peer(peer) == ESP_OK)
//  {
//    Serial.println("Added Peer!");
//  }
//
//  esp_now_register_recv_cb(OnDataRecv);
//  esp_now_register_send_cb(OnDataSent);
//}
//
//
//void prntmac(const uint8_t *mac_addr) {
//  Serial.print("MAC Address: {0x");
//  for (byte i = 0; i < 6; ++i) {
//    Serial.print(mac_addr[i], HEX);
//    if (i < 5)
//      Serial.print(",0x");
//  }
//  Serial.println("};");
//}
//
//
//void loop()
//{
//  yield();
//}
//
//void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
//{
//  if (strncmp((const char *)data, "Calling Master",14) == 0 ) {
//		Serial.printf("\r\nReceived\t%d Bytes\t%s\n", data_len, data);
//    strcpy((char*)dataToSend, "Greeting from Master");
//
//    esp_err_t sendResult = esp_now_send(slave.peer_addr, dataToSend,sizeof(dataToSend)); // maxDataFrameSize);
//    if (sendResult == ESP_OK) {
//      Serial.print("Success delivering response");
//    } else if (sendResult == ESP_ERR_ESPNOW_NOT_INIT) {
//      // How did we get so far!!
//      Serial.println("ESPNOW not Init.");
//    } else if (sendResult == ESP_ERR_ESPNOW_ARG) {
//      Serial.println("Invalid Argument");
//    } else if (sendResult == ESP_ERR_ESPNOW_INTERNAL) {
//      Serial.println("Internal Error");
//    } else if (sendResult == ESP_ERR_ESPNOW_NO_MEM) {
//      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
//    } else if (sendResult == ESP_ERR_ESPNOW_NOT_FOUND) {
//      Serial.println("Peer not found.");
//    }
//    else if (sendResult == ESP_ERR_ESPNOW_IF) {
//      Serial.println("Interface Error.");
//    }   else {
//      Serial.printf("\r\nNot sure what happened\t%d", sendResult);
//    }
//  } else {
//		Serial.printf("\r\nReceived\t%d Bytes\t%d\n", data_len, data[0]);
//	}
//}
//
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
//{
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//	Serial.print("Data sent to ");
//  prntmac(mac_addr);
//  dataSent = 1; //Sent executed
//}
