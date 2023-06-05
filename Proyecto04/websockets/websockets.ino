#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <string.h>

// Replace with your network credentials
const char *ssid = "TP-Link_FA5C";
const char *password = "83898006";

bool ledState = 0;
const int ledPin = 2;

const int ENA = 13,
          ENB = 12,
          IN1 = 32,
          IN2 = 33,
          IN3 = 25,
          IN4 = 26;

// Setting PWM properties
// MIN PWM -> 160
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 200;

void moveMotors(int values[6]) {
  digitalWrite(IN1, values[0]);
  digitalWrite(IN2, values[1]);
  digitalWrite(IN3, values[2]);
  digitalWrite(IN4, values[3]);
  ledcWrite(0, values[4]);
  ledcWrite(1, values[5]);
}

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void notifyClients() { ws.textAll(String(ledState)); }

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
    Serial.printf("\n");
    moveMotors(values);
    // if (strcmp((char *)data, "toggle") == 0) {
    //   ledState = !ledState;
    //   notifyClients();
    // }
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

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  ledcAttachPin(ENA,0);
  ledcAttachPin(ENB,1);
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

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
  ws.cleanupClients();
  digitalWrite(ledPin, ledState);
}
