
#include <ESP8266WiFi.h>

#define BLUE_LED 2

#define HI_BAUD 115200

// Host ip and port
const uint16_t port = 1025;
const char *host = "";
WiFiClient client;
  
void setup()
{
  // Initialize Serial hw
  Serial.begin(HI_BAUD);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  pinMode(BLUE_LED, OUTPUT);
  
  while(WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(BLUE_LED, LOW);
    delay(2500);
    digitalWrite(BLUE_LED, HIGH);
    delay(2500);
  }
   
}

void loop()
{
 
  unsigned int buff_index = 0;
  char buff[256];
    
    while (!client.connect(host, port))
    {
      digitalWrite(BLUE_LED, LOW);
      delay(250);
      digitalWrite(BLUE_LED, HIGH);
      delay(250);
    }
      
    digitalWrite(BLUE_LED, HIGH);
      
    //Clear the serial buffer before sending data to the client
    while(Serial.available()){
      Serial.read();
    }
    
    while (client.connected())
    {
      if(Serial.available()){
        *(buff+buff_index) = Serial.read();
        
        if(*(buff+buff_index) == '\n'){
          client.write(buff,buff_index);
          memset(buff, 0, buff_index);
          buff_index = 0;
        }
        buff_index++;
        
      } 
    } 
    client.stop();
}
