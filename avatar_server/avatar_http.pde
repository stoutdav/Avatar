const int ledPin =  13;

void setup() {
  Serial.begin(9600);
  Serial.flush();
  pinMode(ledPin, OUTPUT);
}

void loop() {
  char serialCommand = getSerialInput();
  if (serialCommand > 0) {
    blink(serialCommand);
  }
}

char getSerialInput() {
  if (Serial.available() > 0) {
    return Serial.read();
  }
  return 0;
}

void blink (int num) {
  int numTimes = num - 48;
  while (numTimes > 0) {
    Serial.println("Blink " + String(numTimes));
    digitalWrite(ledPin, HIGH);
    delay(150);
    digitalWrite(ledPin, LOW);
    delay(150);
    numTimes--;
  }
}


