#include <Arduino.h>

/*
This is code for the magnet and motion sensor "door-node" of the project.
Core principle is that the magnet sensor wakes up the esp32 when a magnet is removed from it's range (attached to the door)
When the door is closed again the motion sensor checks for movement for a set timer (default 10 seconds)
After the timer it will send what it detected (either "1" for movement or "0" for no movement) to ubidots, then go to sleep.
*/

// Ubidots (ubidots defined library)
#include "UbidotsEsp32Mqtt.h"

// Network variables
const char *UBIDOTS_TOKEN   = "";  // Put here your Ubidots TOKEN
const char *WIFI_SSID       = "";                                     // Put here your Wi-Fi SSID
const char *WIFI_PASS       = "";                                     // Put here your Wi-Fi password
const char *DEVICE_LABEL    = "DoorNode";                             // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL  = "Motion";                               // Put here your Variable label to which data  will be published

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

Ubidots ubidots(UBIDOTS_TOKEN);

// Ubidots functions
void connectionStartUp();
void callback(char *topic, byte *payload, unsigned int length);
void sendVariable(int variable);

// Pins:
const int PIR_PIN = 32; // Motion sensor
const int HES_PIN = 33; // Hall effect sensor
const int LED_PIN = 25;

// Common flags
int timerDelay = 10000; 
unsigned long updateMillis = 0;
unsigned long magnetMillis = 0;

bool person_state = false;
enum DOOR_STATE { OPEN, CLOSED, SHUT }; DOOR_STATE door_state = SHUT;

// Sensor flags 
bool HES_state = false;
bool prev_HES_state = false;
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

void startUp(){ // Declare startup values and run startup functions
  Serial.begin(9600);
  pinMode(PIR_PIN, INPUT);
  pinMode(HES_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  connectionStartUp();
}

void updateSensors(){ // Update sensors (only HES, as HES activates PIR);
  if (!ubidots.connected()){ // Make sure ubidots is connected
    ubidots.reconnect();  // Reconnects if not connected
  }

  if(millis()-updateMillis >= 500){ // Only update every half-second to avoid spam. 
    updateMillis = millis();
    updateHES();
    //updatePIR(); // Uncomment to run motion as well (ONLY FOR DEBUG, WILL GO TO SLEEP AFTER TIMER)
  }
}

// Magnet
void updateHES(){ // Update magnet sensor
  prev_HES_state = HES_state; // Save old magnet state to compare

  HES_state = digitalRead(HES_PIN); // Read magnet state
  //Serial.print("Magnet sensor: "); Serial.println(HES_state);

  if(HES_state == 1){ // If sensor responds "1", door is open.
    Serial.println("Door open");
    door_state = OPEN;
  }
  else if(HES_state == 0 && prev_HES_state == 1){ // If sensor responds "0", after being "1", door was just closed.
    Serial.println("Door closed");
    magnetMillis = millis();
    door_state = CLOSED;
  }

  switch (door_state) { // Switch to manage doorstates
    case OPEN: // If door is open do nothing yet
      Serial.println("Door is open");
      break;
    case CLOSED: // If door was closed, update motion detection sensor.
      Serial.println("Checking for motion");
      updatePIR();
      break;
    case SHUT: // If door is closed and a set timer has passed, send ESP32 to deep sleep. 
      Serial.println("Door is closed");
      goToSleep();
      break;
  }
}

// Motion
void updatePIR(){ // Update motion detection sensor
  int PIR_state = digitalRead(PIR_PIN); // Read motion detection state
  //Serial.print("Motion sensor: "); Serial.println(PIR_state);

  if(millis() - magnetMillis >= 1000){ // Wait one second for door to settle.
    if(PIR_state == 1){ // If motion detected, then someone is inside (unless low chance accident, f.eks something falling over within timer)
      motion_state = true; // Save this for later
      digitalWrite(LED_PIN, 1); // Turn on LED when active
    }
    else{
      digitalWrite(LED_PIN, 0); // Turn off LED when not active
    }
  }

  if(millis() - magnetMillis >= timerDelay){ // After a set timerDelay, check if motion was detected (default: 10000ms = 10 seconds)
    if(motion_state){ // If motion sensor saw motion
      Serial.println("Motion was detected");
      person_state = true; // Save "true" for transmission
    } else { // If motion did not see motion
      Serial.println("Motion was not detected");
      person_state = false; // Save "false" for transmission
    }
    
    sendVariable(motion_state); // Send saved state to ubidots server.

    door_state = SHUT; // Tell program the door is not shut (will start deep-sleep, see switch(door_state))
  }
} 

void goToSleep(){ // Initiates deep sleep
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1); // Tell esp to wake up if it get a 1 on pin 33.

    Serial.println("Timer passed, going to sleep");

    delay(1000); // Wait 1 second to make sure wakeup saves properly

    esp_deep_sleep_start(); // Start deep sleep
}

// Ubidots
void connectionStartUp(){ // Start ubidots connection (ubidots code)
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

void sendVariable(int variable){ // Send variable to ubidots client
  ubidots.add(VARIABLE_LABEL, variable); 
  ubidots.publish(DEVICE_LABEL);
  Serial.println("Motion Sent");
}