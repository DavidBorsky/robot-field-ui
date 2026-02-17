// Left motors (M1 + M4)
#define ENA 5
#define IN1 7
#define IN2 8

// Right motors (M2 + M3)
#define ENB 6
#define IN3 9
#define IN4 11

void setup() {

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Move forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 180);
  analogWrite(ENB, 180);

  delay(3000);   // 3 seconds

  // Stop
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {
}
