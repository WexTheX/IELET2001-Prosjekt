///////////////////////////////////////////////////////////////////////////
// Libraries needed:
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SimpleRotary.h>
#include "UbidotsEsp32Mqtt.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SEALEVELPRESSURE_HPA (1013.25) //define the sea level pressure

const char *UBIDOTS_TOKEN = "";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "SensorHUB";   // Put here your Device label to which data  will be published
const char *TEMP_LABEL = "temp"; // Put here your Variable label to which data  will be published
const char *VENT_LABEL = "vent"; // Put here your Variable label to which data  will be published

///////////////////////////////////////////////////////////////////////////

Ubidots ubidots(UBIDOTS_TOKEN); //defines ubidots as a variable

SimpleRotary rotary(4,2,5); // Pin A, Pin B, Button Pin

Adafruit_BME280 bme; // I2C for the temp sensor

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)


///////////////////////////////////////////////////////////////////////////

esp_now_peer_info_t slave;
int chan; 

enum MessageType {PAIRING, DATA,};
MessageType messageType;

int counter = 0;

typedef struct struct_message_temp { //defines the varibles received from the sensor module
  uint8_t msgType;
  uint8_t id;
  float temp;
  float hum;
} struct_message;

typedef struct struct_message_actuator { //defines the varibles received from the vent module
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


///////////////////////////////////////////////////////////////////////////

enum screenModes{ // enum for the different screen modes
  Temp,
  Vent,
  }; screenModes screenMode = Temp;

const int PUBLISH_FREQUENCY = 10000; // Update rate in milliseconds, the data will be sent every 10 seconds

unsigned long timer = 0; // Timer to count the time between publishing data

struct hubData{ // struct for the adjustable values of the hub
  int setTemp = 20;
  int setVent = 50;
  int lastVent = 0;
} hubD; 
hubData lastHubData;

struct sensorData{ // struct for the sensor data
  int id;
  float temp;
  float hum;
};
sensorData Hub{0, 0, 0};
sensorData Servo{1, 0, 0};
sensorData Outside{2, 0, 0};
sensorData lastHub{0, 0, 0};
sensorData lastServo{1, 0, 0};
sensorData lastOutside{2, 0, 0};

unsigned long screenPrinted = 0; //used to check if the screen has been updated recently

struct ScreenData{ //struct for the info that is displayed on the screen
  String line1;
  String line2;
};
ScreenData screenData{"Temp: " + String(hubD.setTemp) + "C", "Roomtemp: " + String(Hub.temp) + "C"};
ScreenData lastScreenData{"", ""};


///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void printMAC(const uint8_t * mac_addr){ //prints the mac address to the serial monitor
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
    Outside.temp = incomingReadings.temp;
    Outside.hum = incomingReadings.hum;
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

void initESP_NOW(){  // Initialize ESP-NOW

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

void setupWifi() { //Configures the wifi
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



void callback(char *topic, byte *payload, unsigned int length)
{ // Callback for subscribing to the Ubidots MQTT broker, this method will be called everytime the device receives data from Ubidots.
// This function updated the values of the variables in the hubData struct
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
  hubD.setTemp = message.toInt();
  } else if (strcmp(topic, "/v2.0/devices/sensorhub/vent/lv") == 0){
  hubD.setVent = message.toInt();
  }

}

///////////////////////////////////////////////////////////////////////////

void screenPrint(String text, String text2) { //Prints the given text to the screen
  if (text == ""){ //If no text is given, use the last text
    text = screenData.line1;
  } else {
    screenData.line1 = text;
  }
  if (text2 == ""){ //If no text is given, use the last text
    text2 = screenData.line2;
  } else {
    screenData.line2 = text2;
  }
  if (screenData.line1 != lastScreenData.line1 || screenData.line2 != lastScreenData.line2){ //if the text has changed, print it to the screen
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println(text);
    display.setCursor(0, 30);
    display.println(text2);
    display.display();
    lastScreenData = screenData;
  }
}

void changeMode(){ //Changes the screen mode between Temp and Vent
  if (screenMode == Temp){
    screenMode = Vent;
    screenData.line1 = "Vent: " + String(hubD.setVent) + "%";
  } else if (screenMode == Vent){
    screenMode = Temp;
    screenData.line1 = "Temp: " + String(hubD.setTemp) + "C";
  }
}

void screen(){ //Prints the screen

  if (screenMode == Temp){
    screenPrint("Temp: "+String(hubD.setTemp)+"C", screenData.line2);
  } else if (screenMode == Vent){
    screenPrint("Vent: "+String(hubD.setVent)+"%", screenData.line2);
  }
}

void led(){ //Controls the led in the rotary encoder
  
  if (screenMode == Vent){
    int l = map(hubD.setVent, 0, 100, 0, 2);
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
    int l = map(hubD.setTemp, 5, 40, 0, 2);
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

float readTemp(){ //reads the temperature from the sensor and prints it to the screen
  Hub.temp = bme.readTemperature();
  screenData.line2 = "Roomtemp: " + String(Hub.temp) + "C";
  return Hub.temp;
}

void knob(){ //checks the rotary encoder for input and changes the setpoints accordingly

  byte rotation = rotary.rotate(); // 0 = not turning, 1 = CW, 2 = CCW
  byte push = rotary.push(); // 0 = not pushed, 1 = pushed

  if (screenMode == Temp){
    if (rotation == 1){
      hubD.setTemp++;
    } else if (rotation == 2){
      hubD.setTemp--;
    }
    if (push == 1){
      changeMode();
    }
  } 
  
  
  else if (screenMode == Vent){
    if (rotation == 1){
      hubD.setVent++;
    } else if (rotation == 2){
      hubD.setVent--;
    }
    if (push == 1){
      changeMode();
    }
  }

  hubD.setTemp = constrain(hubD.setTemp, 5, 40);
  hubD.setVent = constrain(hubD.setVent, 0, 100);

  screen();
}


void tempLogic(){ //Calculates the setpoint for the vent module based on the temperature difference between the room and the outside

  if (screenMode != Vent){
    int InnOut =  Hub.temp - Outside.temp;
    int WantOut = hubD.setTemp - Outside.temp;
    int delta =  InnOut - WantOut;
    if (delta <= 0){
      hubD.setVent = 0;
    } else if (delta <= 10 && delta > 0){
      hubD.setVent = map(delta, 0, 10, 0, 100);
    } else if (delta > 10){
      hubD.setVent = 100;
    }
  }

}




///////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_AP_STA);
  setupWifi();
  initESP_NOW();

  //ubidots.setDebug(true);  // uncomment this to make debug messages available
  screenData.line2 = "Connecting to wifi";
  chan = WiFi.channel();
  ubidots.connect();
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.subscribeLastValue(DEVICE_LABEL, TEMP_LABEL);
  ubidots.subscribeLastValue(DEVICE_LABEL, VENT_LABEL);

  timer = millis();

  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  Wire.begin();

  unsigned status;
  status = bme.begin(0x76); // I2C address. I2C scanner found 0x76
  if (!status) { //checks if the temp sensor is connected
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
  delay(2000); //delay for the screen to initialize
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text for startup
  display.println("Hello, world!");
  display.display(); 
}

void loop() {  

  knob(); //checks the rotary encoder for input

  led(); //updates the led

  tempLogic(); //checks the temperature difference between the room and the outside and sets the vent setpoint accordingly

  if (millis() - screenPrinted > 5000){ //prints the screen every 5 seconds (is overrided by other screenPrints)
    readTemp();
    screenPrinted = millis();
  }

  if (hubD.lastVent != hubD.setVent){ //sends the vent setpoint to the vent module
    outgoingSetpoints.msgType = DATA;
    outgoingSetpoints.id = 0;
    outgoingSetpoints.percentage = hubD.setVent;
    esp_err_t result = esp_now_send(NULL, (uint8_t *) &outgoingSetpoints, sizeof(outgoingSetpoints)); // NULL means send to all peers
    hubD.lastVent = hubD.setVent;
  }
  
  if (!ubidots.connected()){ //reconnects to ubidots if the connection is lost
      screenData.line2 = "Reconnecting to wifi";
      ubidots.connect();
      ubidots.subscribeLastValue(DEVICE_LABEL, TEMP_LABEL);
      ubidots.subscribeLastValue(DEVICE_LABEL, VENT_LABEL);
    }


    if (millis() - timer > PUBLISH_FREQUENCY){ //publishes the data to ubidots every 10 seconds
      if (int(lastHubData.setTemp) != int(hubD.setTemp)){
        ubidots.add("Temp", hubD.setTemp);
        Serial.println("temp"+String(hubD.setTemp));
        Serial.println("lasttemp"+String(lastHubData.setTemp));
        lastHubData.setTemp = hubD.setTemp;
        ubidots.publish(DEVICE_LABEL);
      }
      if (lastHubData.setVent != hubD.setVent){
        ubidots.add("Vent", hubD.setVent);
        Serial.println("vent"+String(hubD.setVent));
        Serial.println("lastvent"+String(lastHubData.setVent));
        lastHubData.setVent = hubD.setVent;
        ubidots.publish(DEVICE_LABEL);
      }
      if (int(lastHub.temp) != int(Hub.temp)){
        ubidots.add("Roomtemp", Hub.temp);
        Serial.println("roomtemp"+String(Hub.temp));
        Serial.println("lastroomtemp"+String(lastHub.temp));
        lastHub.temp = Hub.temp;
        ubidots.publish(DEVICE_LABEL);
      }
      if (int(lastServo.temp) != int(Servo.temp)){
        ubidots.add("VentTemp", Servo.temp);
        Serial.println("venttemp"+String(Servo.temp));
        Serial.println("lastventtemp"+String(lastServo.temp));
        lastServo.temp = Servo.temp;
        ubidots.publish(DEVICE_LABEL);
      }
      if (int(lastServo.hum) != int(Servo.hum)){
        ubidots.add("VentHum", Servo.hum);
        Serial.println("venthum"+String(Servo.hum));
        Serial.println("lastventhum"+String(lastServo.hum));
        lastServo.hum = Servo.hum;
        ubidots.publish(DEVICE_LABEL);
      }
      if (int(lastOutside.temp) != int(Outside.temp)){
        ubidots.add("OutsideTemp", Outside.temp);
        Serial.println("outsidetemp"+String(Outside.temp));
        Serial.println("lastoutsidetemp"+String(lastOutside.temp));
        lastOutside.temp  = Outside.temp;
        ubidots.publish(DEVICE_LABEL);
      }
      if (int(lastOutside.hum)  != int(Outside.hum)){
        ubidots.add("OutsideHum", Outside.hum);
        Serial.println("outsidehum"+String(Outside.hum));
        Serial.println("lastoutsidehum"+String(lastOutside.hum));
        lastOutside.hum = Outside.hum;
        ubidots.publish(DEVICE_LABEL);
      }

      

      timer = millis();
    }
    ubidots.loop();

  
}
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////