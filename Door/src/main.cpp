#include <Arduino.h>

// Ubidots
#include "UbidotsEsp32Mqtt.h"

// Network variables
const char *UBIDOTS_TOKEN   = "BBFF-0aMsYRBJ5JgWojU2IUuwTByFEYqDqi";         // Put here your Ubidots TOKEN
const char *WIFI_SSID       = "foldy";         // Put here your Wi-Fi SSID
const char *WIFI_PASS       = "aihr8372";         // Put here your Wi-Fi password
const char *DEVICE_LABEL    = "DoorNode"; // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL  = "Motion";   // Put here your Variable label to which data  will be published

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

Ubidots ubidots(UBIDOTS_TOKEN);


// Functions
void connectionStartUp();
void callback(char *topic, byte *payload, unsigned int length);
void sendVariable(int variable);

// Pins:
const int PIR_PIN = 32; // Motion sensor
const int HES_PIN = 33; // Hall effect sensor

// Common flags
int timerDelay = 10000;
unsigned long updateMillis = 0;
unsigned long magnetMillis = 0;

bool person_state = false;
enum DOOR_STATE { OPEN, CLOSED, SHUT }; DOOR_STATE door_state = SHUT;

// Magnet
bool HES_state = false;
bool prev_HES_state = false;

// Motion
bool motion_state = false;

// Function Declarations
void startUp();

void updateSensors();
void updateHES();
void updatePIR();

void goToSleep();


void setup() {
  startUp();
}

void loop() {
  updateSensors();
}

void startUp(){ // Delare startup values
  Serial.begin(9600);
  pinMode(PIR_PIN, INPUT);
  pinMode(HES_PIN, INPUT_PULLUP);

  connectionStartUp();
}

void updateSensors(){ // Update sensors (only HES, as HES activates PIR);
  if (!ubidots.connected()){
    ubidots.reconnect();
  }

  if(millis()-updateMillis >= 500){
    updateMillis = millis();
    updateHES();
    //updatePIR();
  }
}

// Magnet
void updateHES(){ //Internal ESP32 hall effect reading (OLD)
  prev_HES_state = HES_state;

  HES_state = digitalRead(HES_PIN);
  Serial.print("Magnet sensor: "); Serial.println(HES_state);

  if(HES_state == 1){ // If dor was closed, then opened.
    Serial.println("Door open");
    door_state = OPEN;
  }
  else if(HES_state == 0 && prev_HES_state == 1){ // If door was open, then closed.
    Serial.println("Door closed");
    magnetMillis = millis();
    door_state = CLOSED;
  }

  switch (door_state) {
    case OPEN:
      Serial.println("Door is open");
      break;
    case CLOSED:
      Serial.println("Checking for motion");
      updatePIR();
      break;
    case SHUT:
      Serial.println("Door is closed");
      goToSleep();
      break;
  }
}

// Motion
void updatePIR(){
  int PIR_state = digitalRead(PIR_PIN);
  Serial.print("Motion sensor: "); Serial.println(PIR_state);

  if(millis() - magnetMillis >= 1000){ // Wait one second for door to settle.
    if(PIR_state == 1){ // If motion detected
      motion_state = true;
    }
  }

  if(millis() - magnetMillis >= timerDelay){
    if(motion_state){
      Serial.println("Motion was detected");
      person_state = true;
    } else {
      Serial.println("Motion was not detected");
      person_state = false;
    }
    
    sendVariable(motion_state);

    door_state = SHUT;
  }
} 

void goToSleep(){
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);

    Serial.println("Timer passed, going to sleep");

    delay(1000);

    esp_deep_sleep_start();
}

// Ubidots
void connectionStartUp(){
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void sendVariable(int variable){
  ubidots.add(VARIABLE_LABEL, variable);
  ubidots.publish(DEVICE_LABEL);
}