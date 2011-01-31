// Digital Pin usage for reference: (Directions from the the perspective of the bot)
// 0 - Right Position Controller
// 1 - Left Position Controller
// 4 - Front Ping Sensor
// 8 - PS2 Controller Clock
// 9 - PS2 Controller Command
// 10 - PS2 Controller Attention
// 11 - PS2 Controller Data

#include <PS2X_lib.h>

const int RIGHT_CONTROLLER_PIN = 0;
const int LEFT_CONTROLLER_PIN = 1;

const int FRONT_PING_SENSOR_PIN = 4;

const int PS2_CLOCK_PIN = 8;
const int PS2_COMMAND_PIN = 9;
const int PS2_ATTENTION_PIN = 10;
const int PS2_DATA_PIN = 11;

const char FORWARD = 'f';
const char BACKWARD = 'b';
const char LEFT = 'l';
const char RIGHT = 'r';

// For PS2 Controller
int error = 0; 
byte type = 0;
byte vibrate = 0;

PS2X ps2x;

void setup() {
	Serial.begin(57600); // Serial from PC was tested with 9600. Will this still work
	
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

void loop() {
	// char command = getSerialInput(); TODO(paul): Handle both serial and ps2 at the same time
	char command = getPS2ControllerInput();
	switch (command) {
		case FORWARD:
			forward();
		break;
		case BACKWARD:
			back();
		break;
		case LEFT:
			left();
		break;
		case RIGHT:
			right();
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


void forward() {
	
 
}

void back() {

}

void left() {

}

void right() {

}






