// Digital Pin usage for reference: (Directions from the the perspective of the bot)
// 0 - Serial Rx to PC
// 1 - Serial Tx to PC
// 3 - Position Controllers (Left has ID=1 and right has ID=2)
// 4 - Front Ping Sensor

#include <NewSoftSerial.h>

const char DEBUG_OFF = 0;
const char DEBUG_ON = 1;
const char DEBUG_CHATTY = 2;
int debugLevel = DEBUG_OFF;

// Pin definitions for motors
#define MOTOR_TX_RX_PIN 3

// Pin definitions for sensors
#define FRONT_PING_SENSOR_PIN 4

// Character constants for serial interface
const char START_CHAR = '!';
const char STOP_CHAR = '?';

// Character constants for return codes
const char COLLISION_WARNING = 'W'; // takes 3 chars representing distance in cms
const char DEBUG_MESSAGE = '#'; // takes 3 chars representing distance in cms

// Character constants for directional commands
const char FORWARD = 'f';
const char BACKWARD = 'b';
const char LEFT = 'l';
const char RIGHT = 'r';
const char EMERGENCY_STOP = 'S';
const char SMOOTH_STOP = 's';

// Character Constants for parameter setting commands and params for reading commands
const char RAMP_SPEED = 'A'; //A for acceleration
const char MAXIMUM_SPEED = 'M';
const char FORWARD_DISTANCE = 'F';
const char REVERSE_DISTANCE = 'B';
const char ROTATION_DISTANCE = 'R';
const char COLLISION_DISTANCE ='C';

const char READ_PARAM = 'P';  // needs to be followed by param constant from list
const char READ_ALL_PARAMS = 'Y';

// Character constants for system commands
const char SET_DEBUG = 'D';
const char RESET = 'T';


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
const byte DefaultRampSpeed = 15; // 5 positions per .25 sec for acceleration/deceleration for the beginning/end of travel. 15 is the default
const unsigned int DefaultMaximumSpeed = 36; // 2 positions per .5 second. 36 is the default;
const unsigned int DefaultForwardDistance = 20;
const unsigned int DefaultReverseDistance = 10;
const unsigned int DefaultRotationDistance = 5;

// Constant definitions for front sonar sensor
const int DefaultFrontCollisionDistance = 5; // in centimeters

// Variables used for motor control
NewSoftSerial MotorSerial(MOTOR_TX_RX_PIN, MOTOR_TX_RX_PIN);

// Configurable Parameters
byte rampSpeed;
unsigned int maximumSpeed;
unsigned int forwardDistance;
unsigned int reverseDistance;
unsigned int rotationDistance;
int frontCollisionDistance;

void setup() {
  resetParametersToDefaults();
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
  setSpeedRampRate();

  // Set maximum speed
  setSpeedMaximum();
}

void setRampSpeed(byte param) {
  smoothStop();
  rampSpeed = param;
  setSpeedRampRate();
};

void setMaximumSpeed(unsigned int param) {
  smoothStop();
  maximumSpeed = param;
  setSpeedMaximum();
};

void setForwardDistance(unsigned int param) {
  forwardDistance = param;
};

void setReverseDistance(unsigned int param) {
  reverseDistance = param;
};

void setRotationDistance(unsigned int param) {
  rotationDistance = param;
};

void setCollisionDistance(long param) {
  frontCollisionDistance = param;
};

void setSpeedRampRate() {
  // Set speed ramp rate (acceleration/deceleration)
  setSpeedRampRate(BothMotors, rampSpeed);
}

void setSpeedMaximum() {
  setSpeedMaximum(BothMotors, maximumSpeed);
}

void setDebug(int param) {
    debugLevel = param;
}

void sendParam(char param) {
  switch(param) {
  case RAMP_SPEED:
    sendParam(RAMP_SPEED, rampSpeed);
    break;
  case MAXIMUM_SPEED:
    sendParam(MAXIMUM_SPEED, maximumSpeed);
    break;
  case FORWARD_DISTANCE:
    sendParam(FORWARD_DISTANCE, forwardDistance);
    break;
  case REVERSE_DISTANCE:
    sendParam(REVERSE_DISTANCE, reverseDistance);
    break;
  case ROTATION_DISTANCE:
    sendParam(ROTATION_DISTANCE, rotationDistance);
    break;
  case COLLISION_DISTANCE:
    sendParam(COLLISION_DISTANCE, frontCollisionDistance);
    break;
  }
}

void loop() {
  checkForForwardCollision();
  if (Serial.available()) {
    String serialCommand = getSerialInput();
    if (serialCommand > 0) {
      log("Serial command received: " + String(serialCommand), DEBUG_CHATTY);
      performCommand(serialCommand);
    }
  }
  delay(100); // delay is necessary or Ping doesn't seem to work properly
}

void checkForForwardCollision() {
  // If bot is about to run into something immediately stop it
  // TODO(paul): Do this more elegantly by decelerating
  // Only check for collision if there is forward motion. i.e. both motors have a negative speed
  // Assumption: Since QPOS has a signed return value this will work.
  if (hasForwardMotion()) {
    long distanceFromObject = ping(FRONT_PING_SENSOR_PIN);
    if (distanceFromObject < frontCollisionDistance) {
      log("Collision imminent: " + String(distanceFromObject) + "cm from object", DEBUG_ON);
      sendCollisionWarning(distanceFromObject);
      emergencyStop();
    }
  }
}

boolean hasForwardMotion() {
  return getSpeed(LeftMotor) * getSpeed(RightMotor) > 0;
}

void performCommand(String command) {
  char baseCommand = command[0];
  int param = command.substring(1, 4).toInt();
  log("Command Received: " + command + " Base Command: " + String(baseCommand) + " Param: " + String(param), DEBUG_ON);
  switch (baseCommand) {
  case EMERGENCY_STOP:
    emergencyStop();
    break;
  case SMOOTH_STOP:
    smoothStop();
    break;
  case FORWARD:
    travelNumberOfPositions(BothMotors, -1 * forwardDistance);
    break;
  case BACKWARD:
    travelNumberOfPositions(BothMotors, 1 * reverseDistance);
    break;
  case LEFT:
    rotate(1 * rotationDistance);
    break;
  case RIGHT:
    rotate(-1 * rotationDistance);
    break;
  case RAMP_SPEED:
    setRampSpeed(param);
    break;
  case MAXIMUM_SPEED:
    setMaximumSpeed(param);
    break;
  case FORWARD_DISTANCE:
    setForwardDistance(param);
    break;
  case REVERSE_DISTANCE:
    setReverseDistance(param);
    break;
  case ROTATION_DISTANCE:
    setRotationDistance(param);
    break;
  case COLLISION_DISTANCE:
    setCollisionDistance(param);
    break;
  case SET_DEBUG:
    setDebug(param);
    break;
  case RESET:
    softReset();
    resetParametersToDefaults();
    sendAllParams();
    break;
  case READ_PARAM:
    sendParam(param);
    break;
  case READ_ALL_PARAMS:
    sendAllParams();
    break;
  }
}

String getSerialInput() {
  char command[4]; // 4 char commands + string end
  delay(100);
  char val = Serial.read();
  if (val == START_CHAR) {
    log("Start Char Rcvd", DEBUG_CHATTY);
    int charsRead = 0;
    while (Serial.available()) {
      val = Serial.read();
      if (val == STOP_CHAR) {
        log("Stop Char Rcvd", DEBUG_CHATTY);
        command[charsRead++] = '\0';
        log("Received Command: " + String(command), DEBUG_CHATTY);
        return command;
      } 
      command[charsRead] = val;
      charsRead++;
    }
  }
  return command;
}

// Rotate right or left by the number of encoder positions specified. Commands are cumulative.
// Right if position < 0
// Left if position > 0
// approx 4.5 degrees per position
void rotate(int positions) {
  log("Rotating " + String(positions) + " positions", DEBUG_ON);
  travelNumberOfPositions(LeftMotor, positions);
  travelNumberOfPositions(RightMotor, -1 * positions);
}

// Immediate stop without deceleration
void emergencyStop() {
  log("Initating emergency stop", DEBUG_ON);
  clearPosition(BothMotors);
}

// Smooth stop with deceleration. Not cumulative.
void smoothStop() {
  log("Initiating smooth stop", DEBUG_ON);
  travelNumberOfPositions(BothMotors, 0);
}

// Move forward or backward by the number of encoder positions specified. Commands are cumulative.
// Forward if position < 0
// Backward if position > 0
// Approx 1.33 cm per position
void travelNumberOfPositions(byte motorId, int positions) {
  log("Travelling " + String(positions) + " positions for motorId(s): " + (int)motorId, DEBUG_ON);
  issueMotorCommand(TRVL, motorId, positions);
}

void clearPosition(byte motorId) {
  log("Clearing positions for motorId(s): " + String((int)motorId), DEBUG_ON);
  issueMotorCommand(CLRP, motorId);
}

int getPosition(byte motorId) {
  log("Getting position for motorId(s): " + String((int)motorId), DEBUG_CHATTY);
  int position = queryMotor(QPOS, motorId);
  log("Response for position for motorId(s): " + String((int)motorId) + " is: " + String(position), DEBUG_CHATTY);

  return position;
}

int getSpeed(byte motorId) {
  log("Getting speed for motorId(s): " + String((int)motorId), DEBUG_CHATTY);
  int speed = queryMotor(QSPD, motorId);
  log("Response for speed for motorId(s): " + String((int)motorId) + " is: " + String(speed), DEBUG_CHATTY);

  return speed;
}

boolean hasArrived(byte motorId, byte tolerance) {
  log("Checking arrival for motorId(s): " + String((int)motorId) + " with tolerance: " + String((int)motorId), DEBUG_CHATTY);
  boolean hasArrived = queryMotor(CHFA, motorId, tolerance);
  log("Response for has arrived for motorId(s): " + String((int)motorId) + " is: " + String(hasArrived), DEBUG_CHATTY);

  return hasArrived;
}

void setOrientationAsReversed(byte motorId) {
  log("Reversing orientation for motorId(s): " + String((int)motorId), DEBUG_CHATTY);
  issueMotorCommand(SREV, motorId);
}

void setSpeedRampRate(byte motorId, byte rampSpeed) {
  log("Setting speed ramp rate for motorId(s): " + String((int)motorId) + " to: " + String((int)rampSpeed), DEBUG_ON);
  issueMotorCommand(SSRR, motorId, rampSpeed);
}

void setSpeedMaximum(byte motorId, int maximumSpeed) {
  log("Setting maximum speed for motorId(s): " + String((int)motorId) + " to: " + String(maximumSpeed), DEBUG_ON);
  issueMotorCommand(SMAX, motorId, maximumSpeed);
}

void setTransmissionDelay(byte motorId, byte delay) {
  log("Setting transmission delay for motorId(s): " + String((int)motorId) + " to: " + String((int)delay), DEBUG_CHATTY);
  issueMotorCommand(STXD, motorId, delay);
}

void issueMotorCommand(byte command, byte motorId) {
  byte fullCommand = command + motorId;
  log("Issuing single byte command: 0x"  + String(fullCommand, HEX), DEBUG_CHATTY);
  sendToMotorSerial(fullCommand);
}

void issueMotorCommand(byte command, byte motorId, byte param) {
  byte fullCommand = command + motorId;
  log("Issuing two byte command: 0x"  + String(fullCommand, HEX) + ", param: " + String((int)param), DEBUG_CHATTY);
  sendToMotorSerial(fullCommand);
  sendToMotorSerial(param);
}

void issueMotorCommand(byte command, byte motorId, int param) {
  byte fullCommand = command + motorId;
  byte high = highByte(param);
  byte low =  lowByte(param);
  log("Issuing three byte command: 0x"  + String(fullCommand, HEX) + ", param:" + String(param), DEBUG_CHATTY);
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
  setMotorPinToTx();
  MotorSerial.print(byteToSend);
}

// Used for QPOS - Query Position, QSPD - Query Speed
int queryMotor(byte command, byte motorId) {
  issueMotorCommand(command, motorId);
  setMotorPinToRx();
  // Do we need to pause?
  byte high = MotorSerial.read();
  byte low = MotorSerial.read();

  log("Received Response high byte: " + String(high, HEX) + " Low byte: " + String(low, HEX), DEBUG_CHATTY);
  return (int)word(high, low);
}

// Only used for CHFA - Check Arrival
boolean queryMotor(byte command, byte motorId, byte tolerance) {
  issueMotorCommand(command, motorId, tolerance);
  setMotorPinToRx();
  // Do we need to pause?
  byte response = MotorSerial.read();

  log("Received Response: " + String(response, HEX), DEBUG_CHATTY);
  return response == Arrived; 
}

void setMotorPinToTx() {
  setPinToTx(MOTOR_TX_RX_PIN);
}

void setPinToTx(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void setMotorPinToRx() {
  setPinToRx(MOTOR_TX_RX_PIN);
}

void setPinToRx(int pin) {
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

void log(String message, int logLevel) {
  if(debugLevel >= logLevel) {
    Serial.print(START_CHAR);
    Serial.print(DEBUG_MESSAGE);
    Serial.print(message);
    Serial.print(STOP_CHAR);
  }
}

void sendCollisionWarning(int distanceFromObject) {
  sendWarning(COLLISION_WARNING, String(distanceFromObject));
}

void sendWarning(String returnCode, String params) {
  Serial.print(START_CHAR);
  Serial.print(returnCode);
  Serial.print(params);
  Serial.print(STOP_CHAR);
}

void softReset() {
  clearPosition(BothMotors);
  clearPosition(BothMotors);
  clearPosition(BothMotors);
}

void resetParametersToDefaults() {
  rampSpeed = DefaultRampSpeed;
  maximumSpeed = DefaultMaximumSpeed;
  forwardDistance = DefaultForwardDistance;
  reverseDistance = DefaultReverseDistance;
  rotationDistance = DefaultRotationDistance;
  frontCollisionDistance = DefaultFrontCollisionDistance;
}

void sendAllParams() {
  sendParam(RAMP_SPEED, rampSpeed);
  sendParam(MAXIMUM_SPEED, maximumSpeed);
  sendParam(FORWARD_DISTANCE, forwardDistance);
  sendParam(REVERSE_DISTANCE, reverseDistance);
  sendParam(ROTATION_DISTANCE, rotationDistance);
  sendParam(COLLISION_DISTANCE, frontCollisionDistance);
}

void sendParam(char param, int value) {
  Serial.print(START_CHAR);
  Serial.print(param);
  Serial.print(String(value));
  Serial.print(STOP_CHAR);
}





