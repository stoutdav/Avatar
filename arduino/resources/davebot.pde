/*================================================= 
DaveBot 1.0
===================================================*/  
/* Arduino-based telepresence robot controller
 * version 1.0 6/10
 */
 
#include <NewSoftSerial.h>              //Load Software serial library (from http://arduiniana.org/libraries/NewSoftSerial/)
#include <Servo2.h>                     //Must use old servo library, part of the Arduino 16 distribution, now known as Servo2. Servos must be on pins 9 and 10.

#define MOTOR_PIN  5	                // Signal pin on both motor controller boards <--> Arduino digital pin 5
                                        // Note: Left motor board is jumpered to ID=1; right to ID=2
#define SONAR_PIN  6	                // Sinal pin on "Ping" sonar sensor board <--> Arduino digital pin 6
#define LBUMP_PIN  7                    // Left bumper sensor <--> Arduino pin digital 7
#define RBUMP_PIN  8                    // Right bumper sensor <--> Arduino pin digital 8
#define PAN_SERVO_PIN 9                 // Signal line of pan servo <--> Arduino digital pin 9
#define TILT_SERVO_PIN 10               // Signal line of tilt servo <--> Arduino digital pin 10
#define NULL_PIN   1                    // Software UART RX pin (not used)
#define LED_PIN 13                      // LED indicated built into the Arduino board

Servo panServo;                          
Servo tiltServo;                          
int step = 5;
const int panInit = 90;
const int tiltInit = 100;
const int panMax = 145;                 //Adjust Max and Min so that the servos don't groan at limit points
const int panMin = 35;
const int tiltMax = 170;
const int tiltMin = 35;
int panPos = panInit;
int tiltPos = tiltInit;
int i = 0;
byte clear_command = 0x28;
byte reverse_left_side_command = 0x31;
byte reverse_right_side_command = 0x32;
byte go_command = 0x20;
byte go_left_wheel_command = 0x21;
byte go_right_wheel_command = 0x22;
byte set_ramp_command = 0x48;
byte distance_high_byte;
byte distance_low_byte;
byte ramp_speed = 5;                    //Accel/decel. ramp speed
char cmd;
int  distance;                          //distance to travel (feet)
int  ticks;                             //distance to travel (raw encoder units)
boolean eStopped = false;               //True if we just halted after bumper hit
long duration;                          //Ping sensor round-trip travel time
long feet;                              //Ping sensor range estimate

//Serial connection to motor controller boards
NewSoftSerial MOTOR_UART(NULL_PIN,MOTOR_PIN); // output to MOTOR_PIN; input not used

void setup()
{
  pinMode(LED_PIN, OUTPUT);               //Blink the LED so you know the Arduino is resetting
  for (i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }

  Serial.begin(9600);	                  // Initialize hardware serial port, sets data rate for comm. to netbook to 9600 bps
 
 //Servo setup stuff
 
  panServo.attach(PAN_SERVO_PIN);         // attaches the servo to the servo object 
  tiltServo.attach(TILT_SERVO_PIN);       // attaches the servo to the servo object 

  panServo.write(panInit);                //Center the camera to start
  tiltServo.write(tiltInit);

//Motor control setup

  MOTOR_UART.begin(19200);             	  // init soft UART. Controller boards operate at 19.2kbits/sec

  MOTOR_UART.print(clear_command);        //Clear any spurious command send during power-on of Arduino

  MOTOR_UART.print(reverse_left_side_command);  //Mandatory or this side wont work right
  
  MOTOR_UART.print(set_ramp_command);
  MOTOR_UART.print(ramp_speed);

  pinMode(RBUMP_PIN, INPUT);              // set right bumpter pin 1 input
  digitalWrite(RBUMP_PIN, HIGH);          // turn on pullup resistor
  
  pinMode(LBUMP_PIN, INPUT);              // set left bumpter pin to input
  digitalWrite(LBUMP_PIN, HIGH);          // turn on pullup resistor

}

void loop()
{
  if (digitalRead(RBUMP_PIN) == LOW)          //Bumped into something
  {
    if (eStopped == false)
    {
      MOTOR_UART.print(clear_command);        //Stop motors immediately (no decelleration ramping)
      eStopped = true;
      Serial.println ("*Right bumper hit!");
    }
  }
    if (digitalRead(LBUMP_PIN) == LOW)        //Bumped into something
  {
    if (eStopped == false)
    {
      MOTOR_UART.print(clear_command);        //Stop motors immediately (no decelleration ramping)
      eStopped = true;
      Serial.println ("*Left bumper hit!");
    }
  }
 
  if (Serial.available() > 0)
  {                                  // read the incoming byte
    cmd = Serial.read();
    
    feet = Ping();                   // report back the ping distance

    Serial.print("*Ping distance: ");
    Serial.println (feet);
    
    switch (cmd)
    {

    case '1':                      //Go 1 foot forward
        Translate(-23);  
      break;
    
    case '2':                      //Go 2 feet forward
      Translate(-46);
    break;
    
    case '3':                      //Go 3 feet forward
      Translate(-69);
    break;
   
    case 'b':                      //Go 1 foot back
      Translate(23);
    break;
    
    case 'l':                      //(lower case L) Rotate robot left a jog
      Rotate(2);
    break;
    
    case 'r':                      //Rotate robot right a jog
      Rotate(-2);
    break;
    
    case 'g':                      //Rotate robot left (gauche) 45 degrees
      Rotate(10);
    break;
    
    case 'd':                      //Rotate robot right (droit) 45 degrees
      Rotate(-10);
    break;
    
    case 'L':                      //Rotate robot left 90 degrees
      Rotate(20);
    break;
    
    case 'R':                      //Rotate robot right 90 degrees
      Rotate(-20);
    break;
    
    case 'T':                      //tilt camera up one step
      tiltPos = tiltServo.read();       //Read current position (0 to 180 range)
      if (tiltPos <= (tiltMax-step))
      {
        tiltServo.write(tiltPos+step);
      }
    break;
    
    case 't':                      //Tilt camera down one step
      tiltPos = tiltServo.read();       //Read current position (0 to 180 range)
      if (tiltPos >= (tiltMin + step))
      {
        tiltServo.write(tiltPos-step);
      }
    break;
    
    case 'P':                      //Pan camera to the left one step
      panPos = panServo.read();         //Read current position (0 to 180 range)
      if (panPos <= (panMax-step))
      {
        panServo.write(panPos+step);
      }
    break;
    
    case 'p':                      //Pan camera to the right one step
      panPos = panServo.read();         //Read current position (0 to 180 range)
      if (panPos >= (panMin + step))
      {
        panServo.write(panPos-step);
      }
    break;
    
     case 'C':                     //Center camera
      panServo.write(panInit);
      tiltServo.write(tiltInit);
    break;
       
    case 'S':                   //Stop
      MOTOR_UART.print(clear_command);        //Stop motors immediately (no decelleration ramping)
    break;
    }
  }
}

//Routines for robot motion

void Translate (int ticks)                    //Move robot fwd (negative) or reverse (positive) a given number of encoder ticks
{
  distance_high_byte = highByte(ticks);
  distance_low_byte = lowByte(ticks);

  MOTOR_UART.print(go_command);
  MOTOR_UART.print(distance_high_byte);
  MOTOR_UART.print(distance_low_byte);
}

void Rotate (int ticks)                       //Rotate robot so that each wheel moves (fwd and rev) by a given number of encoder ticks
{
  distance_high_byte = highByte(ticks);
  distance_low_byte = lowByte(ticks);

  MOTOR_UART.print(go_left_wheel_command);
  MOTOR_UART.print(distance_high_byte);
  MOTOR_UART.print(distance_low_byte);
  
  distance_high_byte = highByte(-1 * ticks);
  distance_low_byte = lowByte(-1 * ticks);
  
  MOTOR_UART.print(go_right_wheel_command);
  MOTOR_UART.print(distance_high_byte);
  MOTOR_UART.print(distance_low_byte);
}

//Stuff for Ping sensor
long Ping()
/* See:  

   http://www.arduino.cc/en/Tutorial/Ping
   
   created 3 Nov 2008
   by David A. Mellis
   modified 30 Jun 2009
   by Tom Igoe
 
   This example code is in the public domain.

 */
{
//Routine returns range estimate from Ping sensor (in inches)
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(SONAR_PIN, OUTPUT);
  digitalWrite(SONAR_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SONAR_PIN, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(SONAR_PIN, INPUT);
  duration = pulseIn(SONAR_PIN, HIGH);

  // convert the time into a distance
  feet = microsecondsToFeet(duration);

//  delay(100);

  return(feet);
}

long microsecondsToFeet(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  // Divided by 12 to get feet
  return microseconds / 74 / 2 / 12;
}

//long microsecondsToCentimeters(long microseconds)
//{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
//  return microseconds / 29 / 2;
//}
