///////////////////////////////////////////////////////////////////////////
#include <Arduino.h>

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
const char *VARIABLE_LABEL = "test"; // Put here your Variable label to which data  will be published

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

unsigned long timer;

Ubidots ubidots(UBIDOTS_TOKEN);

TaskHandle_t Ubi;

// Pin A, Pin B, Button Pin
SimpleRotary rotary(4,2,5);

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int temp = 0;
int vent = 0;
int lasttemp = 0;
int lastvent = 0;
float hubtemp = 0;

String screenMode = "Temp";

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

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

void screenPrint(String text, String text2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println(text);
  display.setCursor(0, 30);
  display.println(text2);
  display.display();
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

void UbidotsTask(void *pvParameters)
{
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  for (;;)
  {
    if (!ubidots.connected()){
      ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
      ubidots.reconnect();
    }
    ubidots.add("Temp", temp); // Insert your variable Labels and the value to be sent
    ubidots.add("Vent", vent);
    ubidots.publish(DEVICE_LABEL);

    ubidots.loop();

    hubtemp = bme.readTemperature();
    delay(PUBLISH_FREQUENCY);
    
  }
}

///////////////////////////////////////////////////////////////////////////

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);

  xTaskCreatePinnedToCore(
                    UbidotsTask,   /* Task function. */
                    "Ubi",     /* name of task. */
                    4096,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    tskIDLE_PRIORITY,           /* priority of the task */
                    &Ubi,      /* Task handle to keep track of created task */
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

  // put your main code here, to run repeatedly:

  screen();

  led();

  byte i;

  // 0 = not turning, 1 = CW, 2 = CCW
  i = rotary.rotate();

  if ( i == 1 ) {
    if (screenMode == "Temp"){
      temp++;
    } else if (screenMode == "Vent"){
      vent++;
    }
  }

  if ( i == 2 ) {
    if (screenMode == "Temp"){
      temp--;
    } else if (screenMode == "Vent"){
      vent--;
    }
  }

  byte j = rotary.push();
	
	if ( j == 1 ){
    if (screenMode == "Temp"){
      screenMode = "Vent";
      screenPrint("Vent: " + String(vent) + "%");
    } else if (screenMode == "Vent"){
      screenMode = "Temp";
      screenPrint("Temp: " + String(temp) + "C");
    }
	}
  
  
}
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////