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

int temp = 0;
int vent = 0;
int lasttemp = 0;
int lastvent = 0;
float hubtemp = 0;

unsigned long screenPrinted = 0;

enum screenModes{
  Temp,
  Vent,
  sleepMode
  }; screenModes screenMode = Temp;

String screenLine1 = "Temp: " + String(temp) + "C";
String screenLine2 = "Roomtemp: " + String(hubtemp) + "C";

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

float readTemp(){
  hubtemp = bme.readTemperature();
  screenLine2 = "Roomtemp: " + String(hubtemp) + "C";
  return hubtemp;
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

  temp = constrain(temp, 0, 100);
  vent = constrain(vent, 0, 100);

  screen();
}

void loop2(void *pvParameters)
{
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  screenLine2 = "Connecting to wifi";
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  for (;;)
  {

    if (!ubidots.connected()){
      screenLine2 = "Reconnecting to wifi";
      ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
      ubidots.reconnect();
    }

    ubidots.add("Temp", temp); // Insert your variable Labels and the value to be sent
    ubidots.add("Vent", vent);
    ubidots.add("Roomtemp", hubtemp);
    ubidots.publish(DEVICE_LABEL);

    ubidots.loop();

    delay(PUBLISH_FREQUENCY);
    //Try removing the publush frequency delay and see if it works / add the publish_frequency delay to the ubidots loop
  }
}





///////////////////////////////////////////////////////////////////////////

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);

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

  // put your main code here, to run repeatedly:

  knob();

  led();

  if (millis() - screenPrinted > 5000){
    readTemp();
    screenPrinted = millis();
  }
  

  
}
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////