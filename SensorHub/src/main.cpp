///////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// Replace with your network credentials (STATION)
/*const char* ssid = "Midjoskyen";
const char* password = "ArneErBest";*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#include <SimpleRotary.h>

#include "UbidotsEsp32Mqtt.h"

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

///////////////////////////////////////////////////////////////////////////

const char *UBIDOTS_TOKEN = "BBFF-0aMsYRBJ5JgWojU2IUuwTByFEYqDqi";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "foldy";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "aihr8372";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "SensorHUB";   // Put here your Device label to which data  will be published
const char *TEMP_LABEL = "temp"; // Put here your Variable label to which data  will be published
const char *VENT_LABEL = "vent"; // Put here your Variable label to which data  will be published

const int PUBLISH_FREQUENCY = 10000; // Update rate in milliseconds

unsigned long timer = 0;

Ubidots ubidots(UBIDOTS_TOKEN);

TaskHandle_t Loop2;

// Pin A, Pin B, Button Pin
SimpleRotary rotary(4,2,5);

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int temp = 20;
int vent = 50;
float roomtemp = 0;
float servoTemp = 0;
float servoHum = 0;
float outsideTemp = 0;
float outsideHum = 0;
int lastVent = 0;

unsigned long screenPrinted = 0;

enum screenModes{
  Temp,
  Vent,
  sleepMode
  }; screenModes screenMode = Temp;

String screenLine1 = "Temp: " + String(temp) + "C";
String screenLine2 = "Roomtemp: " + String(roomtemp) + "C";

esp_now_peer_info_t slave;
int chan; 

enum MessageType {PAIRING, DATA,};
MessageType messageType;

int counter = 0;

typedef struct struct_message_temp {
  uint8_t msgType;
  uint8_t id;
  float temp;
  float hum;
} struct_message;

typedef struct struct_message_actuator {
  uint8_t msgType;
  uint8_t id;
  float percentage;
} struct_message_actuator;

typedef struct struct_pairing {       // new structure for pairing
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
} struct_pairing;

struct_message_temp incomingReadings;
struct_message_actuator outgoingSetpoints;
struct_pairing pairingData;

// ---------------------------- esp_ now -------------------------
void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

bool addPeer(const uint8_t *peer_addr) {      // add pairing
  memset(&slave, 0, sizeof(slave));
  const esp_now_peer_info_t *peer = &slave;
  memcpy(slave.peer_addr, peer_addr, 6);
  
  slave.channel = chan; // pick a channel
  slave.encrypt = 0; // no encryption
  // check if the peer exists
  bool exists = esp_now_is_peer_exist(slave.peer_addr);
  if (exists) {
    // Slave already paired.
    Serial.println("Already Paired");
    return true;
  }
  else {
    esp_err_t addStatus = esp_now_add_peer(peer);
    if (addStatus == ESP_OK) {
      // Pair success
      Serial.println("Pair success");
      return true;
    }
    else 
    {
      Serial.println("Pair failed");
      return false;
    }
  }
} 

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success to " : "Delivery Fail to ");
  printMAC(mac_addr);
  Serial.println();
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print(len);
  Serial.print(" bytes of data received from : ");
  printMAC(mac_addr);
  Serial.println();
  StaticJsonDocument<1000> root;
  String payload;
  uint8_t type = incomingData[0];       // first message byte is the type of message 
  switch (type) {
  case DATA :                           // the message is data type
    memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    // create a JSON document with received data and send it by event to the web page
    root["id"] = incomingReadings.id;
    root["temperature"] = incomingReadings.temp;
    root["humidity"] = incomingReadings.hum;
    serializeJson(root, payload);
    Serial.print("event send :");
    serializeJson(root, Serial);
    // REPLACE WITH MQTT/UBIDOTS:
    //events.send(payload.c_str(), "new_readings", millis());
    if (incomingReadings.id = 1) {
      servoTemp = incomingReadings.temp;
      servoHum = incomingReadings.hum;
    } else if (incomingReadings.id = 2){
      outsideTemp = incomingReadings.temp;
      outsideHum = incomingReadings.hum;
    }
    Serial.println();
    break;
  
  case PAIRING:                            // the message is a pairing request 
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    Serial.println(pairingData.msgType);
    Serial.println(pairingData.id);
    Serial.print("Pairing request from: ");
    printMAC(mac_addr);
    Serial.println();
    Serial.println(pairingData.channel);
    if (pairingData.id > 0) {     // do not replay to server itself
      if (pairingData.msgType == PAIRING) { 
        pairingData.id = 0;       // 0 is server
        // Server is in AP_STA mode: peers need to send data to server soft AP MAC address 
        WiFi.softAPmacAddress(pairingData.macAddr);   
        pairingData.channel = chan;
        Serial.println("send response");
        esp_err_t result = esp_now_send(mac_addr, (uint8_t *) &pairingData, sizeof(pairingData));
        addPeer(mac_addr);
      }  
    }  
    break; 
  }
}

void initESP_NOW(){
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    else {
      Serial.println("ESP-NOW Initialized");
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
} 

void setupWifi() {
  Serial.println();
  Serial.print("Server MAC Address:  ");
  Serial.println(WiFi.macAddress());

  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);
  // Set device as a Wi-Fi Station
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }

  Serial.print("Server SOFT AP MAC Address:  ");
  Serial.println(WiFi.softAPmacAddress());

  chan = WiFi.channel();
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
}



///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void callback(char *topic, byte *payload, unsigned int length)
{
  String message = "";
  Serial.print("UBI: [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    message = message + (char)payload[i];
  }
  Serial.println(message);
  if (strcmp(topic, "/v2.0/devices/sensorhub/temp/lv") == 0){
  temp = message.toInt();
  } else if (strcmp(topic, "/v2.0/devices/sensorhub/vent/lv") == 0){
  vent = message.toInt();
  }

}

void screenPrint(String text, String text2) { //Increase the text size
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  if (text == ""){
    text = screenLine1;
  } else {
    screenLine1 = text;
  }
  display.println(text);
  display.setCursor(0, 30);
  if (text2 == ""){
    text2 = screenLine2;
  } else {
    screenLine2 = text2;
  }
  display.println(text2);
  display.display();
}

void changeMode(){
  if (screenMode == Temp){
    screenMode = Vent;
    screenLine1 = "Vent: " + String(vent) + "%";
  } else if (screenMode == Vent){
    screenMode = Temp;
    screenLine1 = "Temp: " + String(temp) + "C";
  }
}

void screen(){

  if (screenMode == Temp){
    screenPrint("Temp: "+String(temp)+"C", screenLine2);
  } else if (screenMode == Vent){
    screenPrint("Vent: "+String(vent)+"%", screenLine2);
  }
}

void led(){
  
  if (screenMode == Vent){
    int l = map(vent, 0, 100, 0, 2);
    if (l == 0){
      digitalWrite(18, LOW);
      digitalWrite(19, HIGH);
    } else if (l == 1){
      digitalWrite(18, HIGH);
      digitalWrite(19, HIGH);
    } else if (l == 2){
      digitalWrite(18, HIGH);
      digitalWrite(19, LOW);
    }
  }
  if (screenMode == Temp){
    int l = map(temp, 5, 40, 0, 2);
    if (l == 0){
      digitalWrite(18, HIGH);
      digitalWrite(19, LOW);
    } else if (l == 1){
      digitalWrite(18, HIGH);
      digitalWrite(19, HIGH);
    } else if (l == 2){
      digitalWrite(18, LOW);
      digitalWrite(19, HIGH);
    } 
  }
}

float readTemp(){
  roomtemp = bme.readTemperature();
  screenLine2 = "Roomtemp: " + String(roomtemp) + "C";
  return roomtemp;
}

void knob(){

  byte rotation = rotary.rotate(); // 0 = not turning, 1 = CW, 2 = CCW
  byte push = rotary.push(); // 0 = not pushed, 1 = pushed

  if (screenMode == Temp){
    if (rotation == 1){
      temp++;
    } else if (rotation == 2){
      temp--;
    }
    if (push == 1){
      changeMode();
    }
  } 
  
  
  else if (screenMode == Vent){
    if (rotation == 1){
      vent++;
    } else if (rotation == 2){
      vent--;
    }
    if (push == 1){
      changeMode();
    }
  }

  temp = constrain(temp, 5, 40);
  vent = constrain(vent, 0, 100);

  screen();
}

void loop2(void *pvParameters)
{
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  screenLine2 = "Connecting to wifi";
  ubidots.connect();
  chan = WiFi.channel();
  ubidots.connect();
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, TEMP_LABEL);
  ubidots.subscribeLastValue(DEVICE_LABEL, VENT_LABEL);

  struct ubidotsData
  {
    float temp = temp;
    float vent = vent;
    float roomtemp = roomtemp;
    float servoTemp = servoTemp;
    float servoHum = servoHum;
    float outsideTemp = outsideTemp;
    float outsideHum = outsideHum;
  }; ubidotsData lastData;
  


  for (;;)
  {

    if (!ubidots.connected()){
      screenLine2 = "Reconnecting to wifi";
      ubidots.connect();
      ubidots.reconnect();
      ubidots.subscribeLastValue(DEVICE_LABEL, TEMP_LABEL);
      ubidots.subscribeLastValue(DEVICE_LABEL, VENT_LABEL);
    }


    if (millis() - timer > PUBLISH_FREQUENCY){
      if (lastData.temp != temp){
        ubidots.add("Temp", temp);
        lastData.temp = temp;
      }
      if (lastData.vent != vent){
        ubidots.add("Vent", vent);
        lastData.vent = vent;
      }
      if (lastData.roomtemp != roomtemp){
        ubidots.add("Roomtemp", roomtemp);
        lastData.roomtemp = roomtemp;
      }
      if (lastData.servoTemp != servoTemp){
        ubidots.add("VentTemp", servoTemp);
        lastData.servoTemp = servoTemp;
      }
      if (lastData.servoHum != servoHum){
        ubidots.add("VentHum", servoHum);
        lastData.servoHum = servoHum;
      }
      if (lastData.outsideTemp != outsideTemp){
        ubidots.add("OutsideTemp", outsideTemp);
        lastData.outsideTemp = outsideTemp;
      }
      if (lastData.outsideHum != outsideHum){
        ubidots.add("OutsideHum", outsideHum);
        lastData.outsideHum = outsideHum;
      }

      ubidots.publish(DEVICE_LABEL);

      timer = millis();
    }
    ubidots.loop();

    //delay(PUBLISH_FREQUENCY);
    //Try removing the publush frequency delay and see if it works / add the publish_frequency delay to the ubidots loop
  }
}





///////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_AP_STA);
  setupWifi();
  initESP_NOW();
  
  // put your setup code here, to run once

  timer = millis();

  xTaskCreatePinnedToCore(
                    loop2,   /* Task function. */
                    "Loop 2",     /* name of task. */
                    4096,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    tskIDLE_PRIORITY,           /* priority of the task */
                    &Loop2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
  

  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  Wire.begin();

  unsigned status;
  status = bme.begin(0x76); // I2C address. I2C scanner found 0x76
  if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("Hello, world!");
  display.display(); 
}

void loop() {  

  // Feels unnecessary
  /* 
  static unsigned long lastEventTime = millis();
  static const unsigned long EVENT_INTERVAL_MS = 5000;
  if ((millis() - lastEventTime) > EVENT_INTERVAL_MS) {
    events.send("ping",NULL,millis());
    lastEventTime = millis();
    readDataToSend();
    esp_now_send(NULL, (uint8_t *) &outgoingSetpoints, sizeof(outgoingSetpoints));
  }
 */


  knob();

  led();

  if (millis() - screenPrinted > 5000){
    readTemp();
    screenPrinted = millis();
  }

  if (lastVent != vent){
    outgoingSetpoints.msgType = DATA;
    outgoingSetpoints.id = 0;
    outgoingSetpoints.percentage = vent;
    esp_err_t result = esp_now_send(NULL, (uint8_t *) &outgoingSetpoints, sizeof(outgoingSetpoints)); // NULL means send to all peers
    lastVent = vent;
  }
  

  
}
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////