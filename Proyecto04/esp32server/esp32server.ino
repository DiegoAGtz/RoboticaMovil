#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include "Adafruit_VL53L0X.h"
#include <string.h>

// Replace with your network credentials
const char *ssid = "TP-Link_FA5C";
const char *password = "83898006";

//const char *ssid = "Black shark 4 Pro";
//const char *password = "12345678";

// Sensor
// Pines por defecto: SCL = 22, SDA = 21
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int ENA = 13,
          ENB = 12,
          IN1 = 32,
          IN2 = 33,
          IN3 = 25,
          IN4 = 26;

// Setting PWM properties
// MIN PWM -> 150
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 200;

void moveMotors(int values[6]) {
  digitalWrite(IN1, values[0]);
  digitalWrite(IN2, values[1]);
  digitalWrite(IN3, values[2]);
  digitalWrite(IN4, values[3]);
  ledcWrite(0, values[5]);
  ledcWrite(1, values[4]);
}

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void notifyClients(String message) { 
  ws.textAll(message);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len &&
      info->opcode == WS_TEXT) {

    int values[6];
    data[len] = 0;
    char* token = strtok((char*)data, ",");
    for (int i = 0; token != NULL; i++) {
      values[i] = atoi(token);
      token = strtok(NULL, ",");
    }
    moveMotors(values);
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(),
                  client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);

  ledcAttachPin(ENA,0);
  ledcAttachPin(ENB,1);
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());
  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", "Hello World");
  });

  // Start server
  server.begin();
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    int distance = measure.RangeMilliMeter;
    Serial.print("Distance (mm): "); 
    Serial.println(distance);
    notifyClients(String(distance));
  }
  ws.cleanupClients();
  delay(500);
}
