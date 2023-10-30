
/* 
  connection.cc: Code for connection between node and dashboard (ubidots)
 */

#include <connection.hh>

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