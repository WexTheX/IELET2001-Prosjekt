#include <Arduino.h>

// Pins:
const int PIR_PIN = 32; // Motion sensor
const int HES_PIN = 33; // Hall effect sensor

// Common flags
int timerDelay = 10000;
unsigned long updateMillis = 0;
unsigned long magnetMillis = 0;

bool person_state = false;

// Magnet
bool magnet_state = false;
bool prev_magnet_state = false;

// Motion
bool motion_state = false;
bool prev_motion_state = false;

void startUp();

void updateSensors();
void updateHES();
void updatePIR();

void setup() {
  startUp();

}

void loop() {
  //updateSensors();
}

void startUp(){
  Serial.begin(9600);
  pinMode(PIR_PIN, INPUT);
  pinMode(HES_PIN, INPUT);
}

void updateSensors(){
  if(millis()-updateMillis >= 0){
    updateMillis = millis();
    updateHES();
    updatePIR();
  }
}

// Magnet
void updateHES(){
  prev_magnet_state = magnet_state;

  int HES_state = digitalRead(HES_PIN);
  Serial.print("Magnet sensor: "); Serial.println(HES_state);

  if(HES_state == 0){ // Magnet detected, door closed
    magnet_state = true;
  } else if (HES_state == 1){ // Magnet not detected, door open
    magnet_state = false;
  }

  if(magnet_state == 0 && prev_magnet_state == 1){ // If door was open, then closed.
    Serial.println("Door closed");
    magnetMillis = millis();
    updatePIR();
  }
}

// Motion
void updatePIR(){
  prev_motion_state = motion_state;

  int PIR_state = digitalRead(PIR_PIN);
  Serial.print("Motion sensor: "); Serial.println(PIR_state);

    if(PIR_state == 1){ // Motion detected
    motion_state = true;
  }

  if(millis() - magnetMillis >= timerDelay){
    if(motion_state){
      Serial.println("Motion was detected");
      person_state = true;
    } else {
      Serial.println("Motion was not detected");
      person_state = false;
    }
    // TODO: Send data, person_state

    Serial.println("Timer passed, going to sleep");
    // TODO: Sleep
  }
}