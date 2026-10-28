void setup() {
  pinMode(A4, OUTPUT);
}

void loop() {
  pinMode(A4, OUTPUT);
  digitalWrite(A4, LOW);   // device pulls the line down
  delayMicroseconds(10);
  pinMode(A4, INPUT);      // device releases - resistor pulls it up
  delayMicroseconds(10);
}
