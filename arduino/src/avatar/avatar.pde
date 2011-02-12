// Digital Pin usage for reference: (Directions from the the perspective of the bot)
// 0 - Serial Rx to PC
// 1 - Serial Tx to PC
// 3 - Position Controllers (Left has ID=1 and right has ID=2)
// 4 - Front Ping Sensor

#include <NewSoftSerial.h>

// Uncomment this for debugging output
#define AVATAR_DEBUG

// Pin definitions for motors
#define MOTOR_PIN 3

// Pin definitions for sensors
#define FRONT_PING_SENSOR_PIN 4

// Character constants for directional commands
const char FORWARD = 'f';
const char BACKWARD = 'b';
const char LEFT = 'l';
const char RIGHT = 'r';
const char EMERGENCY_STOP = 'S';
const char SMOOTH_STOP = 's';

// Constant definitions for Motor/Position Controllers:
// http://www.parallax.com/Portals/0/Downloads/docs/prod/motors/27906-PositionClrKit-v1.1.pdf
// See: http://www.ieee.org/netstorage/spectrum/articles/oct10/DaveBot_Arduino_Sketch.txt for much of the inspiration
const byte QPOS = 0x08;           //Query Position
const byte QSPD = 0x10;           //Query Speed
const byte CHFA = 0x18;           //Check for Arrival
const byte TRVL = 0x20;           //Travel Number of Positions
const byte CLRP = 0x28;           //Clear Position
const byte SREV = 0x30;           //Set Orientation as Reversed
const byte STXD = 0x38;           //Set TX Delay
const byte SMAX = 0x40;           //Set Speed Maximum
const byte SSRR = 0x48;           //Set Speed Ramp Rate
const byte LeftMotor  = 0x01;    //ID for left motor
const byte RightMotor = 0x02;    //ID for right motor
const byte BothMotors = 0x00;    //ID for both motrs
const byte Arrived = 0xFF;
const byte NotArrive = 0x00;

// Configuration for motor controllers
// According to Parallax documentation position controller have 36 positions per rotation or .5" of linear travel with 6" tires
const byte RampSpeed = 15; // 5 positions per .25 sec for acceleration/deceleration for the beginning/end of travel. 15 is the default
const int MaximumSpeed = 2; // 2 positions per .5 second. 36 is the default;
const int ForwardDistance = 20;
const int BackwardDistance = 10;
const int RotationDistance = 5;

// Constant definitions for front sonar sensor
const long MinFrontDistanceFromObject = 5; // in centimeters

// Variables used for motor control
NewSoftSerial MotorSerial(MOTOR_PIN, MOTOR_PIN);

void setup() {
  setupPcInterface();
  setupMotorControl();
}

void setupPcInterface() {
  Serial.begin(9600);
  Serial.flush();
}

void setupMotorControl() {
  // Init soft UART. Controller boards operate at 19.2kbits/sec
  MotorSerial.begin(19200);

  // Clear Positions
  clearPosition(BothMotors);

  // Reverse the left motor. Depends on how it's been wired
  setOrientationAsReversed(LeftMotor);

  // Set speed ramp rate (acceleration/deceleration)
  setSpeedRampRate(BothMotors, RampSpeed);

  // Set maximum speed
  setSpeedMaximum(BothMotors, MaximumSpeed);
}

void loop() {
  checkForCollision();
  char serialCommand = getSerialInput();
  if (serialCommand > 0) {
    log("Serial command received: " + String(serialCommand));
    performCommand(serialCommand);
  }
  delay(100); // delay is necessary or Ping doesn't seem to work properly
}

void checkForCollision() {
  // If bot is about to run into something immediately stop it
  // TODO(paul): Do this more elegantly by decelerating
  long distanceFromObject = ping(FRONT_PING_SENSOR_PIN);
  if (distanceFromObject < MinFrontDistanceFromObject) {
    log("Collision imminent: " + String(distanceFromObject) + "cm from object");
    emergencyStop();
  }
}

void performCommand(char command) {
  switch (command) {
  case EMERGENCY_STOP:
	emergencyStop();
	break;
  case SMOOTH_STOP:
	smoothStop();
	break;
  case FORWARD:
    travelNumberOfPositions(BothMotors, -1 * ForwardDistance);
    break;
  case BACKWARD:
    travelNumberOfPositions(BothMotors, 1 * BackwardDistance);
    break;
  case LEFT:
    rotate(1 * RotationDistance);
    break;
  case RIGHT:
    rotate(-1 * RotationDistance);
    break;
  }
}

char getSerialInput() {
  if (Serial.available() > 0) {
    return Serial.read();
  }
  return 0;
}

// Rotate right or left by the number of encoder positions specified. Commands are cumulative.
// Right if position < 0
// Left if position > 0
// approx 4.5 degrees per position
void rotate(int positions) {
  log("Rotating " + String(positions) + " positions");
  travelNumberOfPositions(LeftMotor, positions);
  travelNumberOfPositions(RightMotor, -1 * positions);
}

// Immediate stop without deceleration
void emergencyStop() {
  log("Initating emergency stop");
  clearPosition(BothMotors);
}

// Smooth stop with deceleration. Not cumulative.
void smoothStop() {
  log("Initiating smooth stop");
  travelNumberOfPositions(BothMotors, 0);
}

// Move forward or backward by the number of encoder positions specified. Commands are cumulative.
// Forward if position < 0
// Backward if position > 0
// Approx 1.33 cm per position
void travelNumberOfPositions(byte motorId, int positions) {
  log("Travelling " + String(positions) + " positions for motorId(s): " + (int)motorId);
  issueMotorCommand(TRVL, motorId, positions);
}

void clearPosition(byte motorId) {
  log("Clearing positions for motorId(s): " + String((int)motorId));
  issueMotorCommand(CLRP, motorId);
}

int getPosition(byte motorId) {
  log("Getting position for motorId(s): " + String((int)motorId));
  int position = queryMotor(QPOS, motorId);
  log("Response for position for motorId(s): " + String((int)motorId) + " is: " + String(position));

  return position;
}

int getSpeed(byte motorId) {
  log("Getting speed for motorId(s): " + String((int)motorId));
  int speed = queryMotor(QSPD, motorId);
  log("Response for speed for motorId(s): " + String((int)motorId) + " is: " + String(speed));

  return speed;
}

boolean hasArrived(byte motorId, byte tolerance) {
  log("Checking arrival for motorId(s): " + String((int)motorId) + " with tolerance: " + String((int)motorId));
  boolean hasArrived = queryMotor(CHFA, motorId, tolerance);
  log("Response for has arrived for motorId(s): " + String((int)motorId) + " is: " + String(hasArrived));

  return hasArrived;
}

void setOrientationAsReversed(byte motorId) {
  log("Reversing orientation for motorId(s): " + String((int)motorId));
  issueMotorCommand(SREV, motorId);
}

void setSpeedRampRate(byte motorId, byte rampSpeed) {
  log("Setting speed ramp rate for motorId(s): " + String((int)motorId) + " to: " + String((int)rampSpeed));
  issueMotorCommand(SSRR, motorId, rampSpeed);
}

void setSpeedMaximum(byte motorId, int maximumSpeed) {
  log("Setting maximum speed for motorId(s): " + String((int)motorId) + " to: " + String(maximumSpeed));
  issueMotorCommand(SMAX, motorId, maximumSpeed);
}

void setTransmissionDelay(byte motorId, byte delay) {
  log("Setting transmission delay for motorId(s): " + String((int)motorId) + " to: " + String((int)delay));
  issueMotorCommand(STXD, motorId, delay);
}

void issueMotorCommand(byte command, byte motorId) {
  byte fullCommand = command + motorId;
  log("Issuing single byte command: 0x"  + String(fullCommand, HEX));
  sendToMotorSerial(fullCommand);
}

void issueMotorCommand(byte command, byte motorId, byte param) {
  byte fullCommand = command + motorId;
  log("Issuing two byte command: 0x"  + String(fullCommand, HEX) + ", param: " + String((int)param));
  sendToMotorSerial(fullCommand);
  sendToMotorSerial(param);
}

void issueMotorCommand(byte command, byte motorId, int param) {
  byte fullCommand = command + motorId;
  byte high = highByte(param);
  byte low =  lowByte(param);
  log("Issuing three byte command: 0x"  + String(fullCommand, HEX) + ", param:" + String(param));
  sendToMotorSerial(fullCommand);
  sendToMotorSerial(high);
  sendToMotorSerial(low);
}

// From: http://www.arduino.cc/en/Tutorial/Ping. See for using inches instead of cm.
long ping(int pingPin) {
  long pingDuration; // microseconds
  long distance; // centimeters

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  pingDuration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  distance = microsecondsToCentimeters(pingDuration);
  return distance;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void sendToMotorSerial(byte byteToSend) {
	MotorSerial.print(byteToSend);
}

// Used for QPOS - Query Position, QSPD - Query Speed
int queryMotor(byte command, byte motorId) {
	issueMotorCommand(command, motorId);

	setPinToInput(MOTOR_PIN);
	// Do we need to pause?
	byte high = MotorSerial.read();
	byte low = MotorSerial.read();
	setPinToOutput(MOTOR_PIN);
	
	log("Received Response high byte: " + String(high, HEX) + " Low byte: " + String(low, HEX));
	return (int)word(high, low);
}

// Only used for CHFA - Check Arrival
boolean queryMotor(byte command, byte motorId, byte tolerance) {
	issueMotorCommand(command, motorId, tolerance);
	
	setPinToInput(MOTOR_PIN);
	// Do we need to pause?
	byte response = MotorSerial.read();
	setPinToOutput(MOTOR_PIN);

	log("Received Response: " + String(response, HEX));
	return response == Arrived; 
}

void setPinToInput(int pin) {
	// Get Ready to receive
	digitalWrite(pin, LOW);
	pinMode(pin, INPUT);
}

void setPinToOutput(int pin) {
	pinMode(pin, OUTPUT);
}

void log(String message) {
#ifdef AVATAR_DEBUG
  Serial.println(message);
#endif
}

