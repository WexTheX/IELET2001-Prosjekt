
#ifndef CONNECTION
#define CONNECTION

/* 
  connection.hh: Header for all connection variables and functions
 */

#include "UbidotsEsp32Mqtt.h"

// Network variables
const char *UBIDOTS_TOKEN   = "";         // Put here your Ubidots TOKEN
const char *WIFI_SSID       = "";         // Put here your Wi-Fi SSID
const char *WIFI_PASS       = "";         // Put here your Wi-Fi password
const char *DEVICE_LABEL    = "DoorNode"; // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL  = "Motion";   // Put here your Variable label to which data  will be published

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

Ubidots ubidots(UBIDOTS_TOKEN);


// Functions
void connectionStartUp();
void callback(char *topic, byte *payload, unsigned int length);
void sendVariable(int variable);


#endif CONNECTION