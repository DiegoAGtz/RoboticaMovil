const int ENA = 12,
          ENB = 13,
          IN1 = 33,
          IN2 = 32,
          IN3 = 25,
          IN4 = 26;

// Setting PWM properties
// MIN PWM -> 160
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
  ledcAttachPin(ENA,0);
  ledcAttachPin(ENB,1);
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
}

void fordward() {
  ledcWrite(0, 255);
  ledcWrite(1, 255);
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}

void back() {
  ledcWrite(0, 160);
  ledcWrite(1, 160);
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
}

void right() {
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  ledcWrite(0, 200);
  ledcWrite(1, 160);
}

void left() {
  ledcWrite(0, 160);
  ledcWrite(1, 200);
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}

void stopMotors() {
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
}

void loop () {
  fordward();
  delay(5000);
  back();
  delay(3000);
  right();
  delay(2000);
  left();
  delay(2000);
  stopMotors();
  delay (4000);
}
