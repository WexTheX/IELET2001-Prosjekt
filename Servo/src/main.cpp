// libraries to include
#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wifi.h>
#include <EEPROM.h>

Servo servo;

// Set your Board and Server ID 
#define BOARD_ID 1 // change this to a unique value for each ESP in your project
#define MAX_CHANNEL 13  // maximum WiFi channel number

// limits
#define MIN_ANGLE 0.0 
#define MAX_ANGLE 90.0
#define SERVO_PIN 14 

uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // broadcast address

typedef struct struct_message_actuator { // struct to hold incoming data
  uint8_t msgType; // pairing or data
  uint8_t id; // board id
  float percentage; // percentage to set servo
} struct_message;

typedef struct struct_pairing {
  uint8_t msgType; // pairing or data
  uint8_t id; // board id
  uint8_t macAddr[6]; // mac address of the server
  uint8_t channel; // channel to use
} struct_pairing;

struct_message inData; // object to hold incoming data
struct_pairing pairingData; // object to hold pairing data

enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,}; // enumeration for pairing status
PairingStatus pairingStatus = NOT_PAIRED; // set initial pairing status

enum MessageType {PAIRING, DATA,}; // enumeration for message type
MessageType messageType;

int channel = 1; // initial WiFi channel to use

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;
unsigned long start;                // used to measure pairing time
  

void addPeer(const uint8_t * mac_addr, uint8_t chan){ // add peer to peer list
  esp_now_peer_info_t peer; // declare peer object
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE)); // set WiFi channel
  esp_now_del_peer(mac_addr); // delete peer if already exists
  memset(&peer, 0, sizeof(esp_now_peer_info_t)); // clear peer object
  peer.channel = chan; // set channel
  peer.encrypt = false; // no encryption
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6])); // set peer mac address
  if (esp_now_add_peer(&peer) != ESP_OK){ // add peer and check for success
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6])); // set server address
}

void printMAC(const uint8_t * mac_addr){ // print MAC address
  char macStr[18]; // declare c-string to hold MAC address
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", // format MAC address to readable string
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr); // print MAC address
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) // map function for floats
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { // callback function for sent data
  Serial.print("\r\nLast Packet Send Status:\t"); // print status
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { // callback function for received data
  Serial.print("Packet received from: "); // print sender MAC address
  printMAC(mac_addr);
  Serial.println();
  Serial.printf("data size = %d\n", sizeof(incomingData)); // print data size
  uint8_t type = incomingData[0]; // first byte is message type
  float angle = 0.0; // initialize angle with 0.0
  switch (type) {
  case DATA :      // we received data from server
    memcpy(&inData, incomingData, sizeof(inData)); // copy incoming data to inData object
    angle = mapf(inData.percentage, 0.0, 100.0, MIN_ANGLE, MAX_ANGLE); // converts percentage received to servo angle
    angle = constrain(angle, MIN_ANGLE, MAX_ANGLE); // constrain angle to servo limits
    Serial.printf("Servo angle = %.2f\n", angle); // print angle
    servo.write(angle); // set servo angle
    break;
  case PAIRING :    // we received pairing data from server
    memcpy(&pairingData, incomingData, sizeof(pairingData)); // copy incoming data to pairingData object
    if (pairingData.id == 0) { // check if the message comes from server
      printMAC(mac_addr); // print server MAC address
      Serial.print("Pairing done for "); 
      printMAC(pairingData.macAddr); 
      Serial.printf(" on channel %d in %lu ms\n", pairingData.channel, millis()-start); // print pairing time
      addPeer(pairingData.macAddr, pairingData.channel); // add the server  to the peer list 
      pairingStatus = PAIR_PAIRED;             // set the pairing status
    }
    break;
  }  
}

PairingStatus autoPairing(){ // function to automatically pair with server
  switch(pairingStatus) { // check pairing status
    case PAIR_REQUEST: 
    Serial.printf("Pairing request on channel %d\n", channel); // print channel
 
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE)); // set WiFi channel  
    if (esp_now_init() != ESP_OK) { // initialize ESP-NOW and check for success
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

void setup() { // runs once
  ESP32PWM::allocateTimer(0); // allocate timer for servo
  Serial.begin(115200); // initialize serial port
  Serial.println(); 
  Serial.print("Client Board MAC Address:  "); // print MAC address
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA); // set WiFi mode to station
  WiFi.disconnect(); // disconnect from any network
  start = millis(); // start timer
  pairingStatus = PAIR_REQUEST; // set pairing status
}

void loop() { // runs repeatedly
  if (!servo.attached()) { // attach servo if not already attached
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, 500, 2400);
  }
  autoPairing(); // run auto pairing
}