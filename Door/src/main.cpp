#include <Arduino.h>

// Self made librarys:
#include <connection.cc>

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