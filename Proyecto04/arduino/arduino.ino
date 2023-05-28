const int ENA = 10,
          ENB = 11,
          IN1 = 2,
          IN2 = 3,
          IN3 = 4,
          IN4 = 5;

void setup() {
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
}

void fordward() {
  analogWrite (ENA, 255);
  analogWrite (ENB, 255);
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}

void back() {
  analogWrite (ENA, 128);
  analogWrite (ENB, 128);
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
  analogWrite (ENA, 200);
  analogWrite (ENB, 100);
}

void left() {
  analogWrite (ENA, 50);
  analogWrite (ENB, 150);
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}

void stopMotors() {
  analogWrite (ENA, 0);
  analogWrite (ENB, 0);
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
