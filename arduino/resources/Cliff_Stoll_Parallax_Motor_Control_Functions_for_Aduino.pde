/*
  Cliff Stoll's functions to drive the 
  Parallax Motor Mount & Wheel Kit (#27971) (two motor driven wheels)
  via the Parallax Position Controller (#29319) (two built-in encoder readers)
  using two HB-25 Motor Controllers.(#29144)
  
  The Parallax Motor / Encoder system is excellent - strong, speedy, and a very nicely machined.
  I have mounted two of them onto a square of plywood and it can support a person.
  Very nice hardware, with a good set of software functions.  
  
  Unfortunately, my project outgrew the basic stamp 2.  So I've had to use an Arduino.
 Here's how I interfaced the encoder/motor/controller to the Arduiono, along with
 a sample program and relevent functions.
 
  
  This code is for the Arduino Mega.  It should also work on other Arduino boards.
  However, the Arduino Mega supports four independent serial ports.  
  Other Arduinos typically have only one serial port.  
  
  These are 9 functions to control the Parallax motors through their attached encoders.
  Using these functions, I'm able to precisely position my forklift robot and get the
  location, using an Arduino Mega.  
  
  I've written this code for illustration, not efficiency.  
  Questions or suggestions?  Contact me through http://www.kleinbottle.com
   Best wishes to all!   -Cliff   25 July 2010 
  
  
  Important - how to connect the Parallax Motor Mount & Wheel Kit to the Arduino: 
  
  (1)
  The Parallax motor/wheel / encoder system communicates through a TTL-level serial line
  which is almost the same as RS-232, but whose voltage levels are at TTL (0 to 5 volts, not 0 to 12 volts).
  That is OK, since the Arduino serial output is 0 to +5 volts (TTL level)
  
  (2)
  The Parallax motor/wheel/encoder system communicates at 19200 baud.
  That is OK, since it's easy to set the Arduino serial output to 19200 baud.

  (3)  
  The Parallax motor/wheel/encoder system communicates shares input and output on one wire.
  (It sends and receives data on one wire)
  So 3 wires go to the encoder/motor driver:
     Ground, +5 volts, and bi-directional serial data.
     
  This is strange, and is not immediately compatable with the Arduino serial communications port.
  The Arduino serial ports communicate with 2 wires:
  (send on one wire, receive on a different wire).
  So 4 wires come from each Arduino serial port:
      Ground, +5volts, serial data output, and serial data input.  

  To make the 2 data wires into 1, we will take advantage of the fact that
    the parallax receivers are always high impedence.
    the parallax transmitter is low impedence when transmitting, and then goes to high impedience.
  We will allow the Arduino to send data out through a diode
    and the arduino will "pull up" the line to +5volts through a 10K resistor when nobody is transmitting.
    
  So, to connect the Arduino to the motor system, do the following:
  
  get a 1/4th watt, 10Kohm resistor
  get a small diode (1N4001 is OK, but any small diode will work)
  
  Connect Arduiono ground to Parallax Encoder ground.
  Connect Arduino +5 volts output to Parallax Encoder +5volts input
  Connect Arduini serial transmit output to the Anode of a Diode
  Cathode of the diode connects to Arduino Serial Receive Input.
  Connect 10Kohm resistor from +5Volts to Diode Cathode/Arduino Serial Receive Input
  Connect Arduino Serial Receive Input to Parallax Encoder serial I/O

  Note that you can connect this system to one or more Parallax Encoder/wheel motor sets
                    (so you can control both wheels with a single Arduiono serial port)
                    

  AT                                         AT
ARDUINO                                PARALLAX ENCODER/WHEEL MOTOR UNIT

  ground  ---------------------------+---Parallax Encoder 1 +5 volst in (left wheel)
                                     | 
                                     \----Parallax Encoder 2 Serial I/O (right wheel)
    
  
  +5 V   ----------------------------+---Parallax Encoder 1 +5 volst in (left wheel)
                      |              | 
                      \              \----Parallax Encoder 2 Serial I/O (right wheel)
    (10K resistor)    /
                      \    
                      /  
                      |
 Receive Data --------+--------------+----Parallax Encoder 1 Serial I/O (left wheel)
 arduino mega         |              |    
 pin 15 labeled RX3   |              \----Parallax Encoder 2 Serial I/O (right wheel)
                      |
  Anode (no band)   ______
                     \  /
    (diode, 1N4001)   \/    
Cathode (has band)  ------ 
                      |
                      |  
  Transmit Data ------/
  (arduiono mega
  pin 14, labeled TX3


(For those of you who are not ham radio enthusiasts, a tiny bit of electronics:
 the diode has two ends.  The Cathode is marked with a band.  The Anode does not have a band)
 You can wire the diode and 10K resistor directly on the board - they're very small.
 Or you can use a small breadboard.)
 
  
  When connected like this, every command sent out from the Arduino Serial Output 
  will immediately be received by both the Arduino Serial Input
  and also both Parallax Encoder/Motors.
 
  So right after sending a message from the Arduino to the Encoder/Motor, 
  it's important to throw away that echo of the message.  
  Thanks to the Parallax documentation, 
  for each command, we know the the number of bytes sent and expected to be received 
  So we will receive the entire echoed command and perhaps some more bytes of data.
  We then ignore the bytes that are echoed and only return the bytes that come from the encoder/motor.
   
   
   [as an alternative, the Parallax Encoder can be easily rewired so that it sends/receives data over 2 diferent wires rather than 1 wire.  I have not done this, but it's clear from the Parallax schematic how to do it]
   
   
  Notice that I use the Arduino Mega to send commands on the Serial3 port, on pins 14 and 15.
  
  Creates 9 functions.
  
  2010/July/20  -Cliff Stoll   www.kleinbottle.com
 */
 
 
void setup() {
                                 // notice the Serial3 in next line  change for your arduino!
  extern HardwareSerial Serial3; // initial Arduino Mega number 3 serial port, pins 14 and 15 on Mega board.
  Serial3.begin(19200);  // the encoder controller runs at 19200
  
  Serial.begin(19200);   // this is output serial line monitoring for debugging. use serial monitor if debugging1

  
 // these define the hexadecimal commands for Parallax Motor Controller
 // I'm using the same commands that Parallax prints in their documentation for the 27971 motor controller. 

# define QPOS         0x08            //Query Position
# define QSPD         0x10            //Query Speed
# define CHFA         0x18            //Check for Arrival
# define TRVL         0x20            //Travel Number of Positions
# define CLRP         0x28            //Clear Position
# define SREV         0x30            //Set Orientation as Reversed
# define STXD         0x38            //Set TX Delay
# define SMAX         0x40            //Set Speed Maximum
# define SSRR         0x48            //Set Speed Ramp Rate
# define Left_Wheel   0x01            //Set address for left wheel
# define Right_Wheel  0x02            //Set address for right wheel
# define Both_Wheels  0x00            //Set address for both wheels

}  // end of setup


void loop()
{
   Serial.println(" begin testing the motors") ;
  
  Set_Orientation_As_Reversed(Left_Wheel);   // Reverse the left wheel
                                             // this causes left wheel and right wheel to turn in opposite directions
                                             // which is a good thing on a 2-wheeled robot.
  delay (10);
  
  Clear_Position(Left_Wheel);  // clear left wheel position  
    delay (10);
    
  Clear_Position(Right_Wheel);  // clear right wheel positions
    delay (10);

  Clear_Position(Both_Wheels);  // clear both wheel positions  (just to make sure)
    delay (10);
    
  Set_Speed_Ramp_Rate (Both_Wheels, 15);  // set the acceleration rate (15 is the default anyways)
    delay (10);
  
  Set_Speed_Maximum(Left_Wheel, 22);      // speed of the left wheel 
    delay (10);  
    
  Set_Speed_Maximum(Right_Wheel, 22);    // right wheel speed
    delay (10);  
    
  Travel_Number_Of_Positions(Both_Wheels, 36*5);   // rotate both wheels 5 revolutions
                                                    // encoders have 36 steps per revolution
    delay (10);
    
    Serial.println("");
    
    // at this point, the wheels are either turning or getting ready to turn
    // let's wait until our wheels have spun around five times...
 
   while ( ! Check_For_Arrival(Left_Wheel,3)){  // while check for arrival is false (tolerance of 3 means doesn't have to be exact)
     delay(200);                                // delay until wheel arrives at target encoder location.
 
      Serial.print(" Left wheel position is ") ;
      Serial.print(Query_Position(Left_Wheel));  

      Serial.print(" Left wheel speed is ") ;
      Serial.println(Query_Speed(Left_Wheel));  
   }                                            // repeat this loop until we arrive
 
      Serial.print(" Arrived!  Hooray!  Final position in encoder units is ");
      Serial.println(Query_Position(Left_Wheel));  
      
   while(1) { }  // STOP
}                            // end of main loop



int Check_For_Arrival(byte Wheel, int Tolerance){
  int Returned_Result; 
   Returned_Result = Tell_Motor( CHFA,  Wheel,  Tolerance);   // Tolerance is zero to 255 ... 0 means exact, 255 is sloppy
   return Returned_Result;
}


int Travel_Number_Of_Positions(byte Wheel, int Distance_To_Travel){
  int Returned_Result; 
   Returned_Result = Tell_Motor( TRVL,  Wheel,  Distance_To_Travel);   // Travel number of encoder steps
   return Returned_Result;
}


int Query_Position(byte Wheel){
  int Returned_Result; 
   Returned_Result = Tell_Motor( QPOS,  Wheel,  0);   // query position
   return Returned_Result;
}


int Query_Speed(byte Wheel){
  int Returned_Result; 
   Returned_Result = Tell_Motor( QSPD,  Wheel,  0);   // query speed
   return Returned_Result;
}



void Set_Orientation_As_Reversed (byte Wheel) {
  int Returned_Result;
  Returned_Result = Tell_Motor( SREV,  Wheel,  0);   //  wheel reversing 
}



void Set_Speed_Maximum (byte Wheel, int Speed) {
  int Returned_Result;
  Returned_Result = Tell_Motor( SMAX,  Wheel,  Speed);   //  set max speed.  Speed is an integer 0 to 65535
}




void Set_Speed_Ramp_Rate (byte Wheel, byte Ramp_Rate) {
  int Returned_Result;
  Returned_Result = Tell_Motor( SSRR,  Wheel,  Ramp_Rate);   //  set max speed.  Speed is an integer 0 to 65535
}



void Set_Transmit_Delay (byte Delay) {
  int Returned_Result;
   // parameter Delay is in units of about 4.34 microseconds.  default is 115 (or about 540 microsecs)
  Returned_Result = Tell_Motor( STXD, 0, Delay);   //  set minimum delay before respoihnd to a query
                                                 // note that no wheel is specified.
}



void Clear_Position (byte Wheel) {
  int Returned_Result;
  Returned_Result = Tell_Motor( CLRP,  Wheel,  0);   // Clear Position of one or both wheels
}




int Tell_Motor(byte Command, byte Address, int In_Parameter) {
  
# define DEBUG         0x00            // Debug Print.  Set to FF to debug

  int Number_Bytes_To_Write; 
  int Number_Bytes_To_Read;
  int Number_Bytes_In_Input_Queue;
  byte Output_Buffer[4];
  byte Input_Buffer[4];
  int Returned_Result;
  int Returned_Data;
  
  
  Output_Buffer[0] = 0;
  Output_Buffer[1] = 0;
  Output_Buffer[2] = 0;
  Output_Buffer[3] = 0;
 
  Output_Buffer[0] = Command + Address;
  
 
 //  flush the input buffer
  Serial3.flush();   //empty the input buffer
  
  if (Command == QPOS) {       // Query Position. sends 1 byte Command+wheel. Receive integer
    Number_Bytes_To_Write = 1; // 
    Number_Bytes_To_Read = 2;
     if (DEBUG) {Serial.print("QPOS "); }
   }
  else if (Command == QSPD) {  // Query Position. sends 1 byte Command+wheel. Receive integer
    Number_Bytes_To_Write = 1; // 
    Number_Bytes_To_Read = 2;  // 
      if (DEBUG) {Serial.print("QSPD ");} 
    }
  else if (Command == CHFA) {  // Check for Arrival. sends 1 byte Command+wheel, 1 byte Tolerance. Receive 1 byte return (ether 0 or FF)
    Number_Bytes_To_Write = 2; // 
    Number_Bytes_To_Read = 1;  // 
    Output_Buffer[1] = byte(In_Parameter);  // 1 byte parameter - the tolerance
     if (DEBUG) {Serial.print("CHFA "); }
  }
  else if (Command == TRVL) { // Travel Number of Positions. sends 1 byte Command+wheel, signed integer distance. Receives nothing
    Number_Bytes_To_Write = 3; // 
    Number_Bytes_To_Read = 0;  //
   Output_Buffer[1] = byte(In_Parameter / 256);  // high byte
   Output_Buffer[2] = byte(In_Parameter);        //least significant of 2 bytes
      if (DEBUG) {Serial.print("TRVL ");  }  
  }
  else if (Command == CLRP) {   // clear position.  Command and wheel, nothing returned
    Number_Bytes_To_Write = 1; // 
    Number_Bytes_To_Read = 0;  // 
      if (DEBUG) {Serial.print("CLRP "); }
  }
  else if (Command == SREV) {  // set orientation as reversed.  Command and wheel, nothing returned
    Number_Bytes_To_Write = 1; // 
    Number_Bytes_To_Read = 0;  //
      if (DEBUG) {Serial.print("SREV ");  }
  }
  else if (Command == STXD) {   // set transmit delay. Command and wheel and 1 byte delay.  Nothing returned
    Number_Bytes_To_Write = 2; // 
    Number_Bytes_To_Read = 0;  //
    Output_Buffer[1] = byte(In_Parameter);  // the transmit delay in units of about 4.3 microseconds
      if (DEBUG) {Serial.print("STXD ");  }
  }
  else if (Command == SMAX) {  // set speed maximum.  Command and wheel, then integer speed sent.  Nothing returned
    Number_Bytes_To_Write = 3; // 
    Number_Bytes_To_Read = 0;  //
    Output_Buffer[1] = byte(In_Parameter / 256);  // high byte of the integer speed
    Output_Buffer[2] = byte(In_Parameter);        //least significant byte of integer speed
      if (DEBUG) {Serial.print("SMAX "); } 
  }
  else if (Command == SSRR) {  // set speed ramp rate.  command+weel, then 1 byte rate.  Nothing returned
    Number_Bytes_To_Write = 2; // 
    Number_Bytes_To_Read = 0;  //
    Output_Buffer[1] = byte(In_Parameter);  // Acceleration / deceleration for travfel, in units of positions/0.25sec/sec.  power on defaults to 15. 
     if (DEBUG) {Serial.print("SSRR "); }
  }
  
  if (DEBUG) {  
  Serial.print("Tell_Motor Command= ");
  Serial.print(Command, BIN);
  
  Serial.print(" Address= ");
  Serial.print(Address, BIN);
  
  Serial.print(" Parameter= ");
  Serial.print(In_Parameter, DEC);

  
  Serial.print(" Output_Buffer= ");
  for (int i = 0; i < Number_Bytes_To_Write; i++){
    Serial.print(Output_Buffer[i], HEX);
    Serial.print("");
  }

  Serial.print(" Send this many bytes= ");
  Serial.print(Number_Bytes_To_Write, DEC);
  
  Serial.print(" Rcve this many bytes= ");
  Serial.print(Number_Bytes_To_Read, DEC);

  Serial.println("");
  }
  
  //  DO THE DEED
  //  SEND THE COMMAND to the PARALLSAX ENCODER/WHEEL MOTOR!!!

  for (int i=0; i<Number_Bytes_To_Write; i++) {
     Serial3.print(Output_Buffer[i],BYTE);
  }
  
  
  //   Ok, the command is sent over the serial line.
  delay(100);  // wait a little for things to settle
  
  /// GET THE ECHO and possibly a REPLY
  // (at the start of this big function, we flushed the input buffer)  
  // but even if there is no data from the motor to read,
  // we must read it anyways, since there is the echo of our command
  
  // find the number of bytes that are waiting for us
  Number_Bytes_In_Input_Queue = Serial3.available();  
  
        if (DEBUG) {
  Serial.print("Number_Bytes_In_Input_Queue=");  
  Serial.print(Number_Bytes_In_Input_Queue, DEC);
  
  Serial.print(" input bytes are ");
        }

  // Here, we read the bytes echoed by the encoder/motor and also any extra bytes (like a numeric position)
  for(int i = 0; i<Number_Bytes_In_Input_Queue; i++) {
    Input_Buffer[i] = Serial3.read();   // read each byte that is waiting for us
          if (DEBUG) {Serial.print(Input_Buffer[i],HEX);  
          Serial.print(".");}
  
}


   if(Number_Bytes_To_Read == 0) {   // perhaps this is a command which has no data returned, like set speed.
     Returned_Data = 0;              
   }
   
   else if (Number_Bytes_To_Read == 1) {  // or maybe this command returns one byte of information, like set transmit delay
     // the command caused the encoder/motor to return only 1 byte.
     // so return the topmost byte.
     //  Ignore the first bytes in the Input Buffer, which are the echo of the outgoing command
     Returned_Data = Input_Buffer[Number_Bytes_In_Input_Queue-1];     
   }
   
   else if (Number_Bytes_To_Read == 2) {   //  or a command that returns 2 bytes, like query position
     // this command causes the encoder/motor to return 2 bytes.
     // so return only the topmost two bytes.
     //  Ignore the first bytes in the Input Buffer, which are the echo of the outgoing command
     //  I am doing an 8 bit shift by multiplying by 256. I'm Lazy...
     Returned_Data = (256*Input_Buffer[Number_Bytes_In_Input_Queue-2]) +Input_Buffer[Number_Bytes_In_Input_Queue-1];     
   }
   
       if (DEBUG) {     Serial.print(" Returned Data=");
      Serial.print(Returned_Data, DEC);
      
      Serial.println("");
       }
      return Returned_Data;
      
}

 