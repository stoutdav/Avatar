#include <SoftwareSerial.h>

#define rxPin 2                                            // Defines pin to be used as rx pin for LCD03
#define txPin 3                                            // Defines pin to be used as tx pin for LCD03
#define txrxPin 5                                          // Defines a pin to be used as both rx and tx for the SRF01
#define csrMove 0x02                                       // Byte used to tell LCD03 we wish to move the cursor
#define clrScrn 0x0C                                       // Byte used to clear LCD03 screen
#define csrHide 0x04                                       // Byte used to hide LCD03 cursor
#define srfAddress 0x01                                    // Address of the SFR01
#define getSoft 0x5D                                       // Byte to tell SRF01 we wish to read software version
#define getRange 0x54                                      // Byte used to get range from SRF01
#define getStatus 0x5F                                     // Byte used to get the status of the transducer



SoftwareSerial lcd_03 = SoftwareSerial(rxPin, txPin);      // Sets up software serial port for the LCD03
SoftwareSerial srf_01 = SoftwareSerial(txrxPin, txrxPin);  // Sets up software serial port for the SRF01

void setup(){
  pinMode(rxPin, INPUT);                                   
  pinMode(txPin, OUTPUT); 
  
/* Set the baud rate for serial communication with the SRF01 and the LCD03
Due to inacuracies in the arduinos softwareSerial library read time we have had to raise
the baud rate for the SRF01 to 9700 to get an acurate reading. This is normaly set at 9600. */
  srf_01.begin(9700);                                      
  lcd_03.begin(9600);                                      

  delay(200);                                              // Waits some time to make sure that SRF01 is powered up
 
  lcd_03.print(clrScrn, BYTE ); 
  lcd_03.print("SRF01 Example");                           
  lcd_03.print(csrHide, BYTE);
                               
  
  SRF01_Cmd(srfAddress, getSoft);                          // Calls a function to get the SRF01 software version
  int softVer = srf_01.read();                             // Read software version from SRF01
  lcd_03.print(csrMove, BYTE);                             
  lcd_03.print(0x12, BYTE);                                // Moves the cursor to space 18
  lcd_03.print("V:");                                      
  lcd_03.print(softVer);                                   // Prints the software version to LCD03
}

void loop(){
  
  SRF01_Cmd(srfAddress, getRange);                         // Calls a function to get range from SRF01
  byte highByte = srf_01.read();                           // Get high byte
  byte lowByte = srf_01.read();                            // Get low byte
  int range = ((highByte<<8)+lowByte);                     // Put them together
  lcd_03.print(csrMove, BYTE);                             
  lcd_03.print(0x15, BYTE);                                // Move the cursor to location 21
  lcd_03.print("Range = ");                                
  lcd_03.print(range);                                     // Print range result to the screen
  lcd_03.print("  ");                                      // Print some spaces to the screen to make sure space direcly after the result is clear
  
  SRF01_Cmd(srfAddress, getStatus);                         // Call to a function that checks if the trancducer is locked or unlocked
  byte statusByte = srf_01.read();                          // Reads the SRF01 status, The least significant bit tells us if it is locked or unlocked
  int status = statusByte & 0x01;                           // Get status of lease significan bit
  if(status == 0){                                      
    lcd_03.print(csrMove, BYTE);                            
    lcd_03.print(0x3D, BYTE);                               // Moves the cursor to location 61
    lcd_03.print("Unlocked");                            // Prints the word unlocked followd by a couple of spaces to make sure space after has nothing in
  }
   else {                                      
    lcd_03.print(csrMove, BYTE);                                  
    lcd_03.print(0x3D, BYTE);                               // Moves cursor to location 61
    lcd_03.print("Locked   ");                              // Prints the word locked followd by a couple of spaces to make sure that the space after has nothing in
  }
  
  delay(100);
}

void SRF01_Cmd(byte Address, byte cmd){                     // Function to send commands to the SRF01
  pinMode(txrxPin, OUTPUT);                                 // Set pin to output and send break by sending pin low, waiting 2ms and sending it high again for 1ms
  digitalWrite(txrxPin, LOW);                              
  delay(2);                                               
  digitalWrite(txrxPin, HIGH);                            
  delay(1);                                                
  srf_01.print(Address, BYTE);                              // Send the address of the SRF01
  srf_01.print(cmd, BYTE);                                  // Send commnd byte to SRF01
  pinMode(txrxPin, INPUT);                                  // make input ready for Rx
}

