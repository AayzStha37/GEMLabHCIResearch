#include <ArduinoWebsockets.h>
#include <WiFi.h>

const char* ssid = "Humans Wifi"; // Enter SSID
const char* password = "HBo@2489"; // Enter Password

const char* websockets_server_host = "192.168.4.57";
const uint16_t websockets_server_port = 8080;

using namespace websockets;

WebsocketsClient client;

void onMessage(WebsocketsMessage message) {
    Serial.println("Message from server: " + message.data());
}

void setup() {
    Serial.begin(115200);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi, Connecting to server...");

    // Set up the "Host" header
    client.addHeader("Host", "");

    // Connect to the WebSocket server
    bool connected = client.connect(websockets_server_host, websockets_server_port, "/");

    if (connected) {
        Serial.println("SUCCESS: Connected to server!");
    } else {
        Serial.println("ERROR: Failed to connect to server!");
    }

    client.onMessage(onMessage);
}

void loop() {
    // Let the websockets client check for incoming messages
    if (client.available()) {
        client.poll();
    }
}
