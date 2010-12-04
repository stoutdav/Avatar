#define UP_PIN 2
#define DOWN_PIN 5
#define LED_PIN 6
#define LED_DELAY 200

void setup() {
  Serial.begin(9600);
  pinMode(UP_PIN, INPUT);
  digitalWrite(UP_PIN, HIGH);

  pinMode(DOWN_PIN, INPUT);
  digitalWrite(DOWN_PIN, HIGH);

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  processSerial();
  processButton();
}

void processSerial() {
  if (Serial.available() > 0) {
    char command = Serial.read();


    if('u' == command) {
      up();
    };

    if('d' == command) {
      down();
    };

  }
}


void processButton() {

  boolean upButtonState = digitalRead(2);
  boolean downButtonState = digitalRead(5);

  if (upButtonState == LOW) {
    up();
  }

  if (downButtonState == LOW) {
    down();
  }
}

void up() {
  digitalWrite(LED_PIN, HIGH);
  delay(LED_DELAY);
  digitalWrite(LED_PIN, LOW);
  Serial.print("#Up button pushed!");
  Serial.println();
}

void down() {
  digitalWrite(LED_PIN, HIGH);
  delay(LED_DELAY);
  digitalWrite(LED_PIN, LOW);
  delay(LED_DELAY);
  digitalWrite(LED_PIN, HIGH);
  delay(LED_DELAY);
  digitalWrite(LED_PIN, LOW);
  Serial.print("#Down button pushed!");
  Serial.println();
}







