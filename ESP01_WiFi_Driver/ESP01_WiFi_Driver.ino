
#include <ESP8266WiFi.h>

#define BLUE_LED 2

#define HI_BAUD 115200

// Debug AP info
const char* SSID = "ESP DATA TRANSFER";
const char* pass = "ArdTest123";

// Host ip and port
const uint16_t port = 10000;

ESP8266WebServer server(port);

  
void setup()
{
  // Initialize Serial hw
  Serial.begin(HI_BAUD);
  Serial.println();

  WiFi.softAP(SSID, pass);
  
  server.begin();
  Serial.print("Server started. IP address is: ");
  Serial.println(WiFi.softAPIP());
  
}

void loop()
{
 
  unsigned int buff_index = 0;
  char buff[256];
    
    // while (!client.connect(host, port))
    // {
    //   digitalWrite(BLUE_LED, LOW);
    //   delay(250);
    //   digitalWrite(BLUE_LED, HIGH);
    //   delay(250);
    // }
      
    digitalWrite(BLUE_LED, HIGH);
      
    //Clear the serial buffer before sending data to the client
    while(Serial.available()){
      Serial.read();
    }
    
    // while (client.connected())
    // {
    //   if(Serial.available()){
    //     *(buff+buff_index) = Serial.read();
        
    //     if(*(buff+buff_index) == '\n'){
    //       client.write(buff,buff_index);
    //       memset(buff, 0, buff_index);
    //       buff_index = 0;
    //     }else{
    //       buff_index++;
    //     }
        
        
    //   } 
    // } 
    // server.stop();
}
