///////////////////////////////////////////////////
/* This code controls a temperature sensor       */
/* which sends humidity and temperature data     */
/* to a central hub via ESP-NOW.                 */
///////////////////////////////////////////////////

// libraries to include
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#define BOARD_ID 2 // change this to a unique value for each ESP in your project
#define MAX_CHANNEL 13 // maximum WiFi channel number

#define uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds 
#define TIME_TO_SLEEP  10       // Time ESP32 will go to sleep (in seconds) 

RTC_DATA_ATTR int bootCount = 0; // counter for number of reboots

RTC_DATA_ATTR uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // initialized to broadcast address, saved
                                                                         // in RTC memory to preserve value between deep sleep cycles
Adafruit_BME280 bme; // create BME280 object 

//structs
typedef struct struct_message_temp { // struct to hold outgoing data
  uint8_t msgType; // pairing or data
  uint8_t id; // board id
  float temp; // temperature
  float hum; // humidity
} struct_message;

typedef struct struct_pairing {
    uint8_t msgType; // pairing or data
    uint8_t id; // board id
    uint8_t macAddr[6]; // mac address of the server
    uint8_t channel; // channel to use
} struct_pairing;

struct_message_temp outData; // object to hold outgoing data
struct_pairing pairingData; // object to hold pairing data

//enums
enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,}; // enumeration for pairing status
RTC_DATA_ATTR PairingStatus pairingStatus = NOT_PAIRED; // set initial pairing status

enum MessageType {PAIRING, DATA,}; // enumeration for message type
MessageType messageType;

RTC_DATA_ATTR int channel = 1; // initial WiFi channel to use

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;       // Stores last time temperature was published
unsigned long start;                    // used to measure Pairing time
RTC_DATA_ATTR float previousTemp = 0.0; // used to detect change in temperature
float currentTemp = 0.0;                // used to detect change in temperature

void addPeer(const uint8_t * mac_addr, uint8_t chan){ // add peer to peer list and set channel
  esp_now_peer_info_t peer; // declare peer object
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE)); // set WiFi channel
  esp_now_del_peer(mac_addr); // delete peer if it exists

  memset(&peer, 0, sizeof(esp_now_peer_info_t)); // clear peer data
  peer.channel = chan; // set channel
  peer.encrypt = false; // no encryption
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6])); // set peer MAC address
  if (esp_now_add_peer(&peer) != ESP_OK){ // add peer and check for success
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6])); // set server MAC address
}

void printMAC(const uint8_t * mac_addr){ // print MAC address to Serial
  char macStr[18]; // declare c-string to hold MAC address
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", // format MAC address to readable string
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr); // print MAC address
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { // callback when data is sent
  Serial.print("\r\nLast Packet Send Status:\t"); // print status
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { // callback when data is received
  Serial.print("Packet received from: "); // print mac address of sender
  printMAC(mac_addr);
  Serial.println();
  Serial.print("data size = "); // print data size
  Serial.println(sizeof(incomingData));
  uint8_t type = incomingData[0]; // first byte is type of data
  switch (type) {
  case DATA :      // we received data from server. Nothing happens because tempSensor is a passive component
    break;
  case PAIRING:    // we received pairing data from server
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    if (pairingData.id == 0) { // the message comes from server
      printMAC(mac_addr); // print MAC adress of server
      Serial.print("Pairing done for ");
      printMAC(pairingData.macAddr);
      Serial.print(" on channel " );
      Serial.print(pairingData.channel); // channel used by the server
      Serial.print(" in ");
      Serial.print(millis()-start); // pairing time
      Serial.println("ms");
      addPeer(pairingData.macAddr, pairingData.channel); // add the server to the peer list 
      pairingStatus = PAIR_PAIRED; // set the pairing status
    }
    break;
  }  
}

PairingStatus autoPairing(){ // automatic pairing routine
  switch(pairingStatus) {
    case PAIR_REQUEST:
    Serial.print("Pairing request on channel "  ); // print channel
    Serial.println(channel);
    // set WiFi channel   
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
    }

    // set callback routines
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // set pairing data to send to the server
    pairingData.msgType = PAIRING;
    pairingData.id = BOARD_ID;     
    pairingData.channel = channel;

    // add peer and send pairing request
    addPeer(serverAddress, channel);
    esp_now_send(serverAddress, (uint8_t *) &pairingData, sizeof(pairingData));
    previousMillis = millis();
    pairingStatus = PAIR_REQUESTED;
    break;

    case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis();
    if(currentMillis - previousMillis > 250) {
      previousMillis = currentMillis;
      // time out expired,  try next channel
      channel ++;
      if (channel > MAX_CHANNEL){
         channel = 1;
      }   
      pairingStatus = PAIR_REQUEST;
    }
    break;
  }
  return pairingStatus;
}  

void setup() {
  start = millis(); // start awake timer
  Serial.begin(115200);

  ++bootCount; // increment boot number and print it every reboot
  Serial.println("Boot number: " + String(bootCount));

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // configure deep sleep wake up timer

  // initialize BME280 sensor and read current temperature
  if (!bme.begin(0x76)) { 
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  currentTemp = bme.readTemperature(); 
  if (abs(currentTemp - previousTemp) < 1) { // if temperature changes less than 1 degree since last reading
    Serial.printf("powered on for %lu ms\n", millis() - start); // print awake time
    esp_deep_sleep_start(); // start deep sleep
  }
  previousTemp = currentTemp; // set previous temperature to current temperature
  // Print MAC address to serial
  Serial.print("Client Board MAC Address:  "); 
  Serial.println(WiFi.macAddress());

  // initialize WiFi as station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  pairingStatus = PAIR_REQUEST;
}

void loop() { 
  if (autoPairing() == PAIR_PAIRED) { // start autopairing and check if paired
    // set values to send
    outData.msgType = DATA;
    outData.id = BOARD_ID;
    outData.temp = currentTemp;
    outData.hum = bme.readHumidity();

    esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &outData, sizeof(outData)); // send values to server
    Serial.printf("Powered on for %lu ms\n", millis() - start); // print awake time
    esp_deep_sleep_start(); // start deep sleep
  }
}
