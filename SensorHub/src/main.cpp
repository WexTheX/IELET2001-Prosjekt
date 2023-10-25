#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#include <SimpleRotary.h>

// Pin A, Pin B, Button Pin
SimpleRotary rotary(4,2,5);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int temp = 0;
int vent = 0;
int lasttemp = 0;
int lastvent = 0;

String screenMode = "Temp";

void screenPrint(String text) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println(text);
  display.display(); 
  Serial.println("printed to screen");
}

bool change(){
  if (screenMode == "Temp" && temp == lasttemp){
    return false;
  } else if (screenMode == "Vent" && vent == lastvent){
    return false;
  } else {
    return true;
  }
}

void screen(){

  temp = constrain(temp, 0, 100);
  vent = constrain(vent, 0, 100);

  if (!change()){
    return;
  }

  lasttemp = temp;
  lastvent = vent;

  if (screenMode == "Temp"){
    screenPrint("Temp: " + String(temp) + "C");
  } else if (screenMode == "Vent"){
    screenPrint("Vent: " + String(vent) + "%");
  }
}

void led(){
  
  if (screenMode == "Vent"){
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
  if (screenMode == "Temp"){
    int l = map(temp, 0, 100, 0, 2);
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

void setup() {
  Serial.begin(9600);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

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

  screen();

  led();

  byte i;

  // 0 = not turning, 1 = CW, 2 = CCW
  i = rotary.rotate();

  if ( i == 1 ) {
    Serial.println("CW");
    if (screenMode == "Temp"){
      temp++;
    } else if (screenMode == "Vent"){
      vent++;
    }
  }

  if ( i == 2 ) {
    Serial.println("CCW");
    if (screenMode == "Temp"){
      temp--;
    } else if (screenMode == "Vent"){
      vent--;
    }
  }

  byte j = rotary.push();
	
	if ( j == 1 ){
		Serial.println("Pushed");
    if (screenMode == "Temp"){
      screenMode = "Vent";
      screenPrint("Vent: " + String(vent) + "%");
    } else if (screenMode == "Vent"){
      screenMode = "Temp";
      screenPrint("Temp: " + String(temp) + "C");
    }
	}


}