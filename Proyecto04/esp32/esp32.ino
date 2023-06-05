const int LED = 33;

void setup() {
  // Set pin mode
  pinMode(LED, OUTPUT);
}

void loop() {
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
}
