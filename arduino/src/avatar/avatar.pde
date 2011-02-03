// Digital Pin usage for reference: (Directions from the the perspective of the bot)
// 2 - Right Position Controller
// 3 - Left Position Controller
// 4 - Front Ping Sensor
// 8 - PS2 Controller Clock
// 9 - PS2 Controller Command
// 10 - PS2 Controller Attention
// 11 - PS2 Controller Data

#include <PS2X_lib.h>
#include <NewSoftSerial.h>   

// Uncomment this for debugging output
#define AVATAR_DEBUG

// Pin definitions for motors
#define RIGHT_MOTOR_TX_PIN 2
#define LEFT_MOTOR_TX_PIN 3
#define NULL_RX_PIN 12

// Pin definitions for sensors
#define FRONT_PING_SENSOR_PIN 4

// Pin definitions for PS2 controller
#define PS2_CLOCK_PIN  8
#define PS2_COMMAND_PIN 9
#define PS2_ATTENTION_PIN 10
#define PS2_DATA_PIN 11

// Character constants for directional commands
const char FORWARD = 'f';
const char BACKWARD = 'b';
const char LEFT = 'l';
const char RIGHT = 'r';

// Constant definitions for Motor/Position Controllers:
// http://www.parallax.com/Portals/0/Downloads/docs/prod/motors/27906-PositionClrKit-v1.1.pdf
// See: http://www.ieee.org/netstorage/spectrum/articles/oct10/DaveBot_Arduino_Sketch.txt for much of the inspiration
const byte CLEAR_POSITION = 0x29; //0x28 + 1 for device ID. We have 1 device per bus.
const byte SET_ORIENTATION_AS_REVERSED = 0x31; //0x30 + 1 for device ID. We have 1 device per bus.
const byte TRAVEL_NUMBER_OF_POSITIONS = 0x21; //0x20 + 1 for device ID. We have 1 device per bus.
const byte SET_SPEED_RAMP_RATE = 0x49; //0x48 + 1 for device ID. We have 1 device per bus.
const byte rampSpeed = 5; // 5 positions per .25 sec for acceleration/deceleration for the beginning/end of travel. 15 is the default
const byte SET_SPEED_MAXIMUM = 0x41; //0x40 + 1 for device ID. We have 1 device per bus.
const int maximumSpeed = 2; // 2 positions per .5 second. 36 is the default;

// Constant definitions for front sonar sensor
const long minDistanceFromObject = 5; // in centimeters

// Variables used for PS2 Controller
PS2X ps2x;
int error = 0; 
byte type = 0;
byte vibrate = 0;

// Variables used for motor control
NewSoftSerial rightMotorUart(NULL_RX_PIN, RIGHT_MOTOR_TX_PIN); 
NewSoftSerial leftMotorUart(NULL_RX_PIN, LEFT_MOTOR_TX_PIN); 

void setup() {
  setupPcInterface();
  setupPS2Controller();
  setupMotorControl();
}

void setupPcInterface() {
  Serial.begin(9600); 
}

void setupPS2Controller() {
  // PS2 Controller Setup GamePad(clock, command, attention, data, Pressures?, Rumble?) 
  error = ps2x.config_gamepad(PS2_CLOCK_PIN, PS2_COMMAND_PIN, PS2_ATTENTION_PIN, PS2_DATA_PIN, false, false);
  if(error == 0){
    Serial.println("Found Controller, configured successful");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
  }

  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  // What does this do?
  // Serial.print(ps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch(type) {
  case 0:
    Serial.println("Unknown Controller type");
    break;
  case 1:
    Serial.println("DualShock Controller Found");
    break;
  case 2:
    // Guitar Hero Controller not supported. TODO(paul): error?
    Serial.println("GuitarHero Controller Found");
    break;
  }
}

void setupMotorControl() {
  // Init soft UART. Controller boards operate at 19.2kbits/sec
  rightMotorUart.begin(19200);
  leftMotorUart.begin(19200);

  // Clear Positions
  rightMotorUart.print(CLEAR_POSITION);
  leftMotorUart.print(CLEAR_POSITION);

  // Reverse the left motor. Depends on how it's been wired
  leftMotorUart.print(SET_ORIENTATION_AS_REVERSED);

  // Set speed ramp rate (acceleration/deceleration)
  rightMotorUart.print(SET_SPEED_RAMP_RATE);
  rightMotorUart.print(rampSpeed);
  leftMotorUart.print(SET_SPEED_RAMP_RATE);
  leftMotorUart.print(rampSpeed);	

  // Set maximum speed
  rightMotorUart.print(SET_SPEED_MAXIMUM);
  rightMotorUart.print(highByte(maximumSpeed));
  leftMotorUart.print(SET_SPEED_MAXIMUM);
  leftMotorUart.print(lowByte(maximumSpeed));
}

void loop() {
  checkForCollision();
  char serialCommand = getSerialInput(); 
  char ps2Command = getPS2ControllerInput();
  if (serialCommand > 0) {
    log("Serial command received: " + serialCommand);
    performCommand(serialCommand);
  }
  if (ps2Command > 0) {
    log("PS2 command received: " + ps2Command);
    performCommand(ps2Command);
  }
  delay(100); // delay is necessary or Ping doesn't seem to work properly
}

void checkForCollision() {
  // If bot is about to run into something immediately stop it
  // TODO(paul): Do this more elegantly by decelerating
  long distanceFromObject = ping(FRONT_PING_SENSOR_PIN);
  if (distanceFromObject < minDistanceFromObject) {
    log("Collision imminent: " + String(distanceFromObject) + "cm from object");
    emergencyStop();
  }
}

void performCommand(char command) {
  switch (command) {
  case FORWARD:
    travel(-1);
    break;
  case BACKWARD:
    travel(1);
    break;
  case LEFT:
    rotate(1);
    break;
  case RIGHT:
    rotate(-1);
    break;
  }
}

char getSerialInput() {
  if (Serial.available() > 0) {
    return Serial.read();
  }
  return 0;
}

char getPS2ControllerInput() {
  if(error == 1)
    //skip if no controller found
    return 0;

  ps2x.read_gamepad(); //read controller. no params used because rumble not enabled. Should be called at least once per second

    // If the right analog stick has not been pressed then do nothing.
  if (!ps2x.Button(PSB_R1)) {
    return 0;
  }

  // TODO(paul): handle more than one axis at a time
  int yAxis = map(ps2x.Analog(PSS_RY),  0, 255, -1, 1);
  int xAxis = map(ps2x.Analog(PSS_RX),  0, 255, -1, 1);

  if (yAxis != 0) {
    if (yAxis == 1) {
      return FORWARD;
    }
    if (yAxis == -1) {
      return BACKWARD;
    }
  }

  if (xAxis != 0) {
    if (xAxis == 1) {
      return RIGHT;
    }
    if (xAxis == -1) {
      return LEFT;
    }
  }	
  return 0;	
}

// Move forward or backward by the number of encoder positions specified. Commands are cumulative.
// Forward if position < 0
// Backward if position > 0
// Approx 1.33 cm per position
void travel(int positions) {
  log("Travelling " + String(positions) + " positions");
  byte highByte = highByte(positions);
  byte lowByte = lowByte(positions);

  rightMotorUart.print(TRAVEL_NUMBER_OF_POSITIONS);
  rightMotorUart.print(highByte);
  rightMotorUart.print(lowByte);

  leftMotorUart.print(TRAVEL_NUMBER_OF_POSITIONS);
  leftMotorUart.print(highByte);
  leftMotorUart.print(lowByte);
}

// Rotate right or left by the number of encoder positions specified. Commands are cumulative.
// Right if position < 0
// Left if position > 0
// approx 4.5 degrees per position
void rotate(int positions) {
  log("Rotating " + String(positions) + " positions");
  leftMotorUart.print(TRAVEL_NUMBER_OF_POSITIONS);
  leftMotorUart.print(highByte(positions));
  leftMotorUart.print(lowByte(positions));

  rightMotorUart.print(TRAVEL_NUMBER_OF_POSITIONS);
  rightMotorUart.print(highByte(-1 * positions));
  rightMotorUart.print(lowByte(-1 * positions));
}

// Immediate stop without deceleration
void emergencyStop() {
  log("Initating emergency stop");
  rightMotorUart.print(CLEAR_POSITION);
  leftMotorUart.print(CLEAR_POSITION);
}

// Smooth stop with deceleration. Not cumulative.
void smoothStop() {
  log("Initiating smooth stop");
  rightMotorUart.print(0);
  leftMotorUart.print(0);
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

void log(String message) {
  #ifdef AVATAR_DEBUG
    Serial.println(message);
  #endif
}





