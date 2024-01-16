#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include "HardwareSerial.h"

//SPI pins
// MOSI - 35
// MISO - 37
// SCK - 36
// CS - 14

#define CS_PIN  14

const char* ssid = "APEX"; 
const char* password = "HBo@2489";
const char* PARAM_MESSAGE = "message";

AsyncWebServer server(80);

HardwareSerial Serial_1(1);

void setup() {
    int TX1, RX1;
    Serial.begin(115200);
    pinMode(CS_PIN, OUTPUT);

    RX1   = 49;
    TX1   = 48;
    Serial_1.begin(115200, SERIAL_8N1, RX1, TX1);
    // Initialize SPI
    SPI.begin();

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("SUCCESS: Connected to WiFi.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Listening to client requests...");

    handleEndPoints();
    server.begin();
}

void handleEndPoints(){

  // Recieve message from the flask client
  server.on("/receive_message", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Get the raw content of the request
    String message;
    //printRequestParameters(request);
    // Check if the PARAM_MESSAGE parameter exists in the request
    if (request->hasParam(PARAM_MESSAGE, true)) {
        // Get the value of the PARAM_MESSAGE parameter
        message = request->getParam(PARAM_MESSAGE, true)->value();
        Serial.println("Received message from Flask server: " + message);
        request->send(200, "text/plain", "Message received by ESP32");
        sendDataToDaisy();

    } else {
        Serial.println("Message not found in the request");
        request->send(400, "text/plain", "PARAM_MESSAGE not found in the request");
    }
});

}


void sendDataToDaisy(){
  // digitalWrite(CS_PIN, LOW);
  // // Send a message byte by byte
  // const char* message = "Hello!";
  // for (int i = 0; i < strlen(message); i++) {
  //   SPI.transfer(message[i]);
  // }
 
  //Serial.println("sine");
  Serial_1.write('1');
}

// void printRequestParameters(AsyncWebServerRequest *request) {
//   // Print basic information
//   Serial.print("Request Method: ");
//   Serial.println(request->methodToString());
  
//   Serial.print("Request URI: ");
//   Serial.println(request->url());

//   // Print headers
//   Serial.println("Headers:");
//   int headerCount = request->headers();
//   for (int i = 0; i < headerCount; i++) {
//     AsyncWebHeader* header = request->getHeader(i);
//     Serial.print(header->name());
//     Serial.print(": ");
//     Serial.println(header->value());
//   }
//   Serial.println("POST Parameters:");
//     int postParamsCount = request->params();
//     for (int i = 0; i < postParamsCount; i++) {
//       AsyncWebParameter* param = request->getParam(i);
//       Serial.print(param->name());
//       Serial.print(": ");
//       Serial.println(param->value());
//     }
// }



void loop() {
    // Your ESP32 code here
}
