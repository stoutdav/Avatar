// Digital Pin usage for reference: (Directions from the the perspective of the bot)
// 0 - Serial Rx to PC
// 1 - Serial Tx to PC
// 3 - Position Controllers (Left has ID=1 and right has ID=2)
// 4 - Front Ping Sensor

#include <NewSoftSerial.h>
#include <CmdMessenger.h>
#include <Base64.h>
#include <Streaming.h>

// Pin definitions
#define MOTOR_TX_RX_PIN 3
#define FRONT_PING_SENSOR_PIN 4

// Variables used for motor control
NewSoftSerial MotorSerial(MOTOR_TX_RX_PIN, MOTOR_TX_RX_PIN);

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

// Serial interface
const char FIELD_SEPARATOR = ',';
const char COMMAND_SEPARATOR = ';';
CmdMessenger cmdMessenger = CmdMessenger(Serial, FIELD_SEPARATOR, COMMAND_SEPARATOR);

// Commands sent to PC
enum
{
  COMM_ERROR    = 0, // Lets Arduino report serial port comm error back to the PC (only works for some comm errors)
  ACK           = 1, // Arduino acknowledges cmd was received
  AVATAR_READY = 2, // After opening the comm port, send this cmd 02 from PC to check arduino is ready
  ERR           = 3, // Arduino reports badly formatted cmd, or cmd not recognised
  DEBUG_MESSAGE  = 4,
  FRONT_COLLISION  = 5,
  RAMP_SPEED    = 6, //A for acceleration
  MOTION_MULTIPLIER      = 7,
  ROTATION_DISTANCE = 8,
  COLLISION_DISTANCE = 9,
  RESET        = 10,
  SEND_CMDS_END,
};

// Commands received and handled by arduino
messengerCallbackFunction messengerCallbacks[] =
{
  sendAllParams, //11
  joystick,      //12
  setDebugLevel, //13
  setRampSpeed, //14
  setFrontSensorDistance, //15
  setRotationDistance, //16
  setMotionMultiplier, //17
  reset, //18
  NULL
};

// Constants to represent the direction of motion
const int FORWARD_DIRECTION = -1;
const int REVERSE_DIRECTION = 1;
const int LEFT_ROTATION = 1;
const int RIGHT_ROTATION = -1;

// Debug Constants
const char DEBUG_OFF = 0;
const char DEBUG_ON = 1;
const char DEBUG_CHATTY = 2;

// Parameter Defaults
const unsigned int DefaultMotionMultiplier = 3; //Used to keep motors moving between commands since the Position Controller is used
const unsigned int DefaultRotationDistance = 3;
const int DefaultFrontSensorDistance = 5; // in centimeters;
// According to Parallax documentation position controller have 36 positions per rotation or .5" of linear travel with 6" tires
const byte DefaultRampSpeed = 15; // 5 positions per .25 sec for acceleration/deceleration for the beginning/end of travel. 15 is the default. 255 is max
int debugLevel = DEBUG_OFF;

// Configurable Parameters
byte rampSpeed;
unsigned int motionMultiplier;
unsigned int rotationDistance;
int frontSensorDistance;

void setup() {
  setupPcInterface();
  resetParametersToDefaults();
  setupMotorControl();
}

void setupPcInterface() {
  Serial.begin(115200);   
  cmdMessenger.attach(AVATAR_READY, avatarReady);
  cmdMessenger.attach(unknownCmd);

  // Attach user-defined callback methods
  attachCallbacks(messengerCallbacks);
  avatarReady();
}

void attachCallbacks(messengerCallbackFunction* callbacks) {
  int i = 0;
  int offset = SEND_CMDS_END;
  while(callbacks[i]) {
    cmdMessenger.attach(offset+i, callbacks[i]);
    i++;
  }
}

void loop() {
  checkForForwardCollision();
  cmdMessenger.feedinSerialData();
  delay(100); // delay is necessary or Ping doesn't seem to work properly
}

// Callbacks for handling commands from PC
void avatarReady() {
  cmdMessenger.sendCmd(ACK,"Avatar ready");
}

void unknownCmd() {
  // Default response for unknown commands and corrupt messages
  cmdMessenger.sendCmd(ERR,"Unknown command");
}

void sendAllParams() {
  cmdMessenger.sendCmd(ACK,"SendAllParams Command Received");
  doSendAllParams();
}

void joystick() {
  cmdMessenger.sendCmd(ACK,"Joystick Command Received");
  int params[2];
  int index = 0;
  while (cmdMessenger.available()) {
    char buffer[350] = { 
      '\0'                                                                         };
    cmdMessenger.copyString(buffer, 350);
    if(buffer[0]) {

      params[index] = atoi(buffer);
      index++;
    }
  }
  move(params[0], params[1]);
}

void setDebugLevel() {
  cmdMessenger.sendCmd(ACK,"SetDebugLevel Command Received");
  int params[1];
  while (cmdMessenger.available()) {
    char buffer[350] = { 
      '\0'                                                                         };
    cmdMessenger.copyString(buffer, 350);
    if(buffer[0]) {
      debugLevel = atoi(buffer);
    }    
  }
}

void setRampSpeed() {
  cmdMessenger.sendCmd(ACK,"SetRampSpeed Command Received");
  int params[1];
  while (cmdMessenger.available()) {
    char buffer[350] = { 
      '\0'                                                                         };
    cmdMessenger.copyString(buffer, 350);
    if(buffer[0]) {
      setRampSpeed(atoi(buffer));
    }    
  }
}

void setFrontSensorDistance() {
  cmdMessenger.sendCmd(ACK,"SetFrontSensorDistance Command Received");
  int params[1];
  while (cmdMessenger.available()) {
    char buffer[350] = { 
      '\0'                                                                         };
    cmdMessenger.copyString(buffer, 350);
    if(buffer[0]) {
      frontSensorDistance = atoi(buffer);
    }    
  }
}

void setRotationDistance() {
  cmdMessenger.sendCmd(ACK,"SetRotationDistance Command Received");
  int params[1];
  while (cmdMessenger.available()) {
    char buffer[350] = { 
      '\0'                                                                         };
    cmdMessenger.copyString(buffer, 350);
    if(buffer[0]) {
      rotationDistance = atoi(buffer);
    }    
  }
}

void setMotionMultiplier() {
  cmdMessenger.sendCmd(ACK,"SetMotionMultiplier Command Received");
  int params[1];
  while (cmdMessenger.available()) {
    char buffer[350] = { 
      '\0'                                                                         };
    cmdMessenger.copyString(buffer, 350);
    if(buffer[0]) {
      motionMultiplier = atoi(buffer);
    }    
  }
}

void reset() {
  cmdMessenger.sendCmd(ACK, "Reset Command Received");
  hardReset();
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
}

// Parameter setting

void setRampSpeed(byte param) {
  smoothStop();
  rampSpeed = param;
  setSpeedRampRate();
};

void setSpeedRampRate() {
  // Set speed ramp rate (acceleration/deceleration)
  setSpeedRampRate(BothMotors, rampSpeed);
}

// Send parameters to client
void sendParam(int commandId, int value) {
  String valueStr = String(value);
  char buffer[255] = { 
    '\0'                                     };
  sprintf(buffer, "%i", value);
  cmdMessenger.sendCmd(commandId, buffer);
}

void resetParametersToDefaults() {
  rampSpeed = DefaultRampSpeed;
  motionMultiplier = DefaultMotionMultiplier;
  rotationDistance = DefaultRotationDistance;
  frontSensorDistance = DefaultFrontSensorDistance;
}

void checkForForwardCollision() {
  // If bot is about to run into something immediately stop it
  // TODO(paul): Do this more elegantly by decelerating
  // Only check for collision if there is forward motion. i.e. both motors have a negative speed
  // Assumption: Since QPOS has a signed return value this will work.
  if (hasForwardMotion()) {
    long distanceFromObject = ping(FRONT_PING_SENSOR_PIN);
    if (distanceFromObject < frontSensorDistance) {
      sendCollisionWarning(distanceFromObject);
      emergencyStop();
    }
  }
}

boolean hasForwardMotion() {
  return getSpeed(LeftMotor) > 0 &&  getSpeed(RightMotor) > 0;
}

// Motor commands
void move(int x, int y) {
  //logMsg("Moving: " + String(x) + " y: " + String(y), DEBUG_ON);
  int requestedSpeed = 0;
  int requestedDirection = 0;
  int requestedTurn = 0;
  int requestedTurnDirection = 0;
  requestedTurn = abs(y);
  requestedSpeed = abs(x);

  if (x < 0) {
    requestedDirection = REVERSE_DIRECTION;
  } 
  else {
    requestedDirection = FORWARD_DIRECTION;
  };

  if (y < 0) {
    requestedTurnDirection = LEFT_ROTATION;
  } 
  else {
    requestedTurnDirection = RIGHT_ROTATION;
  };

  // Stopping
  if (requestedSpeed == 0 && requestedTurn == 0) {
    smoothStop();
  };

  // Moving
  if (requestedSpeed > 0 && requestedTurn == 0) {
    setSpeedMaximum(BothMotors, requestedSpeed);
    travelNumberOfPositions(BothMotors, requestedDirection * requestedSpeed * motionMultiplier);
  };

  // Turning
  if (requestedSpeed == 0 && requestedTurn < 0) {
    setSpeedMaximum(BothMotors, requestedTurn);
    rotatePositions(requestedTurnDirection * requestedTurn * rotationDistance);
  };


  // Turning and moving. Not sure how this will actually work.
  if (requestedSpeed > 0 && requestedTurn > 0) {
    setSpeedMaximum(LeftMotor,requestedSpeed + (requestedTurn * requestedTurnDirection));
    setSpeedMaximum(RightMotor, requestedTurn + (requestedTurn * requestedTurnDirection * -1));
    travelNumberOfPositions(BothMotors, requestedDirection * requestedSpeed * motionMultiplier);
  };

  //TODO(paul): Handle other conditions?

}

// Rotate right or left by the number of encoder positions specified. Commands are cumulative.
// Right if position < 0
// Left if position > 0
// approx 4.5 degrees per position
void rotatePositions(int positions) {
  // logMsg("Rotating " + String(positions) + " positions", DEBUG_ON);
  travelNumberOfPositions(LeftMotor, positions);
  travelNumberOfPositions(RightMotor, -1 * positions);
}

// Move forward or backward by the number of encoder positions specified. Commands are cumulative.
// Forward if position < 0
// Backward if position > 0
// Approx 1.33 cm per position
void travelNumberOfPositions(byte motorId, int positions) {
  //  logMsg("Travelling " + String(positions) + " positions for motorId(s): " + (int)motorId, DEBUG_ON);
  issueMotorCommand(TRVL, motorId, positions);
}

// Immediate stop without deceleration
void emergencyStop() {
  //logMsg("Initating emergency stop",  DEBUG_CHATTY);
  clearPosition(BothMotors);
}

// Smooth stop with deceleration. Not cumulative.
void smoothStop() {
  //logMsg("Initiating smooth stop",  DEBUG_ON);
  travelNumberOfPositions(BothMotors, 0);
}

void clearPosition(byte motorId) {
  //logMsg("Clearing positions for motorId(s): " + String((int)motorId),  DEBUG_CHATTY);
  issueMotorCommand(CLRP, motorId);
}


void setOrientationAsReversed(byte motorId) {
  //  logMsg("Reversing orientation for motorId(s): " + String((int)motorId), DEBUG_CHATTY);
  issueMotorCommand(SREV, motorId);
}

void setSpeedRampRate(byte motorId, byte rampSpeed) {
  //  logMsg("Setting speed ramp rate for motorId(s): " + String((int)motorId) + " to: " + String((int)rampSpeed),  DEBUG_CHATTY);
  issueMotorCommand(SSRR, motorId, rampSpeed);
}

void setSpeedMaximum(byte motorId, int maximumSpeed) {
  //  logMsg("Setting maximum speed for motorId(s): " + String((int)motorId) + " to: " + String(maximumSpeed),  DEBUG_ON);
  issueMotorCommand(SMAX, motorId, maximumSpeed);
}

void setTransmissionDelay(byte motorId, byte delay) {
  //  logMsg("Setting transmission delay for motorId(s): " + String((int)motorId) + " to: " + String((int)delay), DEBUG_CHATTY);
  issueMotorCommand(STXD, motorId, delay);
}

void softReset() {
  clearPosition(BothMotors);
  clearPosition(BothMotors);
  clearPosition(BothMotors);
}

void hardReset() {
  softReset();
  resetParametersToDefaults();
  doSendAllParams();
}

void doSendAllParams() {
  sendParam(RAMP_SPEED, rampSpeed);
  sendParam(MOTION_MULTIPLIER, motionMultiplier);
  sendParam(ROTATION_DISTANCE, rotationDistance);
  sendParam(COLLISION_DISTANCE, frontSensorDistance);
}

void issueMotorCommand(byte command, byte motorId) {
  byte fullCommand = command + motorId;
  //  String commandStr = String(fullCommand, HEX);
  //  logMsg("Issuing single byte command: 0x"  + commandStr, DEBUG_CHATTY);
  sendToMotorSerial(fullCommand);
}

void issueMotorCommand(byte command, byte motorId, byte param) {
  byte fullCommand = command + motorId;
  //  String commandStr = String(fullCommand, HEX);
  //  String paramStr = String(param, HEX);
  //  logMsg("Issuing two byte command: 0x"  + commandStr + " param: " + paramStr, DEBUG_CHATTY);
  sendToMotorSerial(fullCommand);
  sendToMotorSerial(param);
}

void issueMotorCommand(byte command, byte motorId, int param) {
  byte fullCommand = command + motorId;
  //  String commandStr = String(fullCommand, HEX);
  byte high = highByte(param);
  byte low =  lowByte(param);
  //  logMsg("Issuing three byte command: 0x"  + commandStr + " param:" + String(param), DEBUG_CHATTY);
  sendToMotorSerial(fullCommand);
  sendToMotorSerial(high);
  sendToMotorSerial(low);
}

void sendToMotorSerial(byte byteToSend) {
  setMotorPinToTx();
  MotorSerial.print(byteToSend);
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

// Motor Queries

int getPosition(byte motorId) {
  //  logMsg("Getting position for motorId(s): " + String((int)motorId), DEBUG_CHATTY);
  int position = queryMotor(QPOS, motorId);
  //  logMsg("Response for position for motorId(s): " + String((int)motorId) + " is: " + String(position), DEBUG_CHATTY);

  return position;
}

int getSpeed(byte motorId) {
  //  logMsg("Getting speed for motorId(s): " + String((int)motorId), DEBUG_CHATTY);
  int speed = queryMotor(QSPD, motorId);
  //  logMsg("Response for speed for motorId(s): " + String((int)motorId) + " is: " + String(speed), DEBUG_CHATTY);

  return speed;
}

boolean hasArrived(byte motorId, byte tolerance) {
  //  logMsg("Checking arrival for motorId(s): " + String((int)motorId) + " with tolerance: " + String((int)motorId), DEBUG_CHATTY);
  boolean hasArrived = queryMotor(CHFA, motorId, tolerance);
  //  logMsg("Response for has arrived for motorId(s): " + String((int)motorId) + " is: " + String(hasArrived), DEBUG_CHATTY);

  return hasArrived;
}

// Used for QPOS - Query Position, QSPD - Query Speed
int queryMotor(byte command, byte motorId) {
  issueMotorCommand(command, motorId);
  setMotorPinToRx();
  // Do we need to pause?
  byte high = MotorSerial.read();
  //  String highStr = String(high, HEX);
  byte low = MotorSerial.read();
  //  String lowStr = String(low, HEX);

  //  logMsg("Received Response high byte: " + highStr + " Low byte: " + lowStr, DEBUG_CHATTY);
  return (int)word(high, low);
}

// Only used for CHFA - Check Arrival
boolean queryMotor(byte command, byte motorId, byte tolerance) {
  issueMotorCommand(command, motorId, tolerance);
  setMotorPinToRx();
  // Do we need to pause?
  byte response = MotorSerial.read();

  //  logMsg("Received Response: " + String(response, HEX), DEBUG_CHATTY);
  return response == Arrived;
}

// Sonar commands

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

void sendCollisionWarning(long distanceFromObject) {
  String message = String(distanceFromObject);
  char buffer[255] = { 
    '\0'                                     };
  message.toCharArray(buffer, message.length());
  cmdMessenger.sendCmd(FRONT_COLLISION, buffer);
}


void logMsg(String message, int level) {
  //  if (debugLevel >= level) {
  //    char buffer[255] = {
  //      '\0'                                     };
  //    message.toCharArray(buffer, message.length());
  //    cmdMessenger.sendCmd(DEBUG_MESSAGE, buffer);
  //  }
}














