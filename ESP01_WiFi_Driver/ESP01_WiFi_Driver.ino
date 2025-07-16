#include <WiFi.h>


// ESP32 SoftAP Network Credentials
const char* ssid = "ESP32-Server";
const char* pass = "medpass123";

// Server Port Number
WiFiServer server(10000);

void setup()
{
  Serial1.begin(115200);
  
  // Creating the Access Point for user connection
  WiFi.softAP(ssid, pass);

  server.begin();
}


void loop()
{
  WiFiClient client = server.available();
  

  if(client)  // Ensure a client was found
  {
  
    while(client.connected())  // While client has active connection with this server
    {
      if(Serial1.available())
      {
        String sensor_data = Serial1.readString();
        unsigned int sensor_data_len = sensor_data.length();
        char buff[sensor_data_len];
        client.write(buff, sensor_data_len);
      }
             
      
    }

    client.stop();
    Serial1.println("Client disconnected...");
  }
}
