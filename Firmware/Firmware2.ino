



// Include Libraries
#include "Arduino.h"
#include "StepperMotor.h"


// Pin Definitions
#define STEPPER_1_PIN_STEP	5
#define STEPPER_1_PIN_DIR	2

#define STEPPER_2_PIN_STEP	6
#define STEPPER_2_PIN_DIR	3

#define STEPPER_3_PIN_STEP	7
#define STEPPER_3_PIN_DIR	4

// CNC Enable Pin
#define CNC_ENABLE  8   //  stepper motor enable , to active pin -> low


// Global variables and defines
#define stepper_1DelayTime  1000
#define stepper_2DelayTime  1000
#define stepper_3DelayTime  1000

#define stepper_AllDelayTime    1000

// Coordinates
float x,y,z;
x = 0;
y = 0;
z = 0;


// object initialization
StepperMotor stepper_1(STEPPER_1_PIN_STEP,STEPPER_1_PIN_DIR);
StepperMotor stepper_2(STEPPER_2_PIN_STEP,STEPPER_2_PIN_DIR);
StepperMotor stepper_3(STEPPER_3_PIN_STEP,STEPPER_3_PIN_DIR);

void setup() {

	// Setup Serial which is useful for debugging
	// Use the Serial Monitor to view printed messages
	Serial.begin(9600);
	while (!Serial); // wait for serial port to connect. Needed for native USB
	Serial.println("start");

	// enable the stepper motor, use .disable() to disable the motor
	stepper_1.enable();
	// set stepper motor speed by changing the delay value, the higher the delay the slower the motor will turn
	stepper_1.setStepDelay(stepper_1DelayTime);

	// enable the stepper motor, use .disable() to disable the motor
	stepper_2.enable();
	// set stepper motor speed by changing the delay value, the higher the delay the slower the motor will turn
	stepper_2.setStepDelay(stepper_2DelayTime);

	// enable the stepper motor, use .disable() to disable the motor
	stepper_3.enable();
	// set stepper motor speed by changing the delay value, the higher the delay the slower the motor will turn
	stepper_3.setStepDelay(stepper_3DelayTime);

	Serial.println("\n \t\t START \n\n");

}

void loop() {
	Serial.println("Current Coordinates:");
	Serial.println("\t X: %f", &x);
	Serial.println("\t Y: %f", &y);
	Serial.println("\t Z: %f", &z);
	Serial.println(" ");

	startUpMenu();

}

char startUpMenu() {
	Serial.println("");

	Serial.println(F("\n Choose : "));
	Serial.println(F("\t (1) Test Motors"));
	Serial.println(F("\t (2) Go to home"));
	Serial.println(F("\t (3) Goto X, Y, Z"));
	Serial.println(F("(menu) send anything else or press on board reset button\n"));


	// Read data from serial monitor if received
	while (Serial.available())
	{
		char c = Serial.read();
		if (isAlphaNumeric(c)) {

			if ( c == '1' ) {
				Serial.println(F("\n Testing the motors: RUNNING"));
				testMotors();
				Serial.println(F(" Testing the motors: DONE"));
			} else if ( c == '2' ) {
				Serial.println(F("\n Going Home: RUNNING"));
				gotoXYZ(0,0,0);
				Serial.println(F(" Going Home: DONE"));
			} else if ( c == '3' ) {
				Serial.println(F("\n Goto:"));
				Serial.println(F("\t X: "));
				Serial.println(F("\t Y: "));
				Serial.println(F("\t Z: "));
				Serial.println(F(" RUNNING"));
				gotoXYZ(x,y,z);
				Serial.println(F("\n (X,Y,Z):(%f,%f,%f) :: DONE"), &x,&y,&z);
			} else {
				Serial.println(F("illegal input!"));
				return('e');
			}

	}



}






// Global variables

float x0, y0, z0;
float xd, yd, zd;
float th1h=0, th1h=0, th1h=0;
float al=100, bl=50, le=280, lf=100;
float X0, Y0, Z0;

float th1, th2, th3;


void XYZ2Thita() {
	al = Al ;
	bl = Bi ;
	le = Le ;
	lf = Lf ;
	xo = x ;
	yo = y ;
	zo = z ;
	a1 = (xo * xo + yo * yo + zo * zo + lf * lf - le * le - al * al + bl * bl - 2 * bl * yo) / (2 * zo) ;
	b1 = (bl - al - yo) / zo ;
	con1 = (al + a1 * b1) * (al + a1 * b1) - (1 + b1 * b1) * (al * al + a1 * a1 - lf * lf) ;
	if (con1 < 0) {
		print(" The chosen point is beyond the work space of the robot for the selected lengths of the parameters.\n") ;
	} else {
		y1a = (-(al + a1 * b1) + math.sqrt(con1)) / (1 + b1 * b1) ;
		y1b = (-(al + a1 * b1) - math.sqrt(con1)) / (1 + b1 * b1) ;
	}

	if (y1a < y1b) {
		z1a = a1 + b1 * y1a ;
		th1 = (180 / math.pi) * (math.atan(z1a / abs(y1a + al))) ;
	} else {
		z1b = a1 + b1 * y1b ;
		th1 = (180 / math.pi) * (math.atan(z1b / abs(y1b + al))) ;
		xo2 = (-xo + math.sqrt(3) * yo) / 2 ;
		yo2 = (-(math.sqrt(3) * xo + yo)) / 2 ;
		zo2 = zo ;
		a2 = (xo2 * xo2 + yo2 * yo2 + zo2 * zo2 + lf * lf - le * le - al * al + bl * bl - 2 * bl * yo2) / (2 * zo2) ;
		b2 = (bl - al - yo2) / zo2 ;
		con2 = (al + a2 * b2) * (al + a2 * b2) - (1 + b2 * b2) * (al * al + a2 * a2 - lf * lf) ;
	}

	if (con2 < 0) {
		messagebox.showinfo("Error", ;
							" The chosen point is beyond the work space of the robot for the selected lengths of the parameters.") ;
		print("The chosen point is beyond the work space of the robot for the selected lengths of the parameters.\n") ;
	} else {
		y2a = (-(al + a2 * b2) + math.sqrt(con2)) / (1 + b2 * b2) ;
		y2b = (-(al + a2 * b2) - math.sqrt(con2)) / (1 + b2 * b2) ;
	}

	if (y2a < y2b) {
		z2a = a2 + b2 * y2a ;
		th2 = (180 / math.pi) * (math.atan(z2a / abs(y2a + al))) ;
	} else {
		z2b = a2 + b2 * y2b ;
		th2 = (180 / math.pi) * (math.atan(z2b / abs(y2b + al))) ;
		xo3 = (-(xo + math.sqrt(3) * yo)) / 2 ;
		yo3 = (math.sqrt(3) * xo - yo) / 2 ;
		zo3 = zo ;
		a3 = (xo3 * xo3 + yo3 * yo3 + zo3 * zo3 + lf * lf - le * le - al * al + bl * bl - 2 * bl * yo3) / (2 * zo3) ;
		b3 = (bl - al - yo3) / zo3 ;
	}

	con3 = (al + a3 * b3) * (al + a3 * b3) - (1 + b3 * b3) * (al * al + a3 * a3 - lf * lf) ;

	if (con3 < 0) {
		messagebox.showinfo("Error", ;
							" The chosen point is beyond the work space of the robot for the selected lengths of the parameters.") ;
		print("The chosen point is beyond the work space of the robot for the selected lengths of the parameters.\n") ;
	} else {
		y3a = (-(al + a3 * b3) + math.sqrt(con3)) / (1 + b3 * b3) ;
		y3b = (-(al + a3 * b3) - math.sqrt(con3)) / (1 + b3 * b3) ;
	}

	if (y3a < y3b) {
		z3a = a3 + b3 * y3a ;
		th3 = (180 / math.pi) * (math.atan(z3a / abs(y3a + al))) ;
	} else {
		z3b = a3 + b3 * y3b ;
		th3 = (180 / math.pi) * (math.atan(z3b / abs(y3b + al))) ;
	}
}

































// define vars for testing menu
const int timeout = 10000;       //define timeout of 10 sec
char menuOption = 0;
long time0;

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup()
{
	// Setup Serial which is useful for debugging
	// Use the Serial Monitor to view printed messages
	Serial.begin(9600);
	while (!Serial) ; // wait for serial port to connect. Needed for native USB
	Serial.println("start");

	// enable the stepper motor, use .disable() to disable the motor
	stepper_1.enable();
	// set stepper motor speed by changing the delay value, the higher the delay the slower the motor will turn
	stepper_1.setStepDelay(stepper_1DelayTime);
	// enable the stepper motor, use .disable() to disable the motor
	stepper_2.enable();
	// set stepper motor speed by changing the delay value, the higher the delay the slower the motor will turn
	stepper_2.setStepDelay(stepper_2DelayTime);
	// enable the stepper motor, use .disable() to disable the motor
	stepper_3.enable();
	// set stepper motor speed by changing the delay value, the higher the delay the slower the motor will turn
	stepper_3.setStepDelay(stepper_3DelayTime);
	menuOption = menu();

}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop()
{


	if(menuOption == '1') {
	// Stepper Motor with EasyDriver #1 - Test Code
	// the higher the time value the slower the motor will run
	stepper_1.step(1, 1000);  // move motor 1000 steps in one direction
	delay(1000);            // short stop
	stepper_1.step(0, 1000);  // move motor 1000 steps in the other dirction
	delay(1000);            //short stop
	}
	else if(menuOption == '2') {
	// Stepper Motor with EasyDriver #2 - Test Code
	// the higher the time value the slower the motor will run
	stepper_2.step(1, 1000);  // move motor 1000 steps in one direction
	delay(1000);            // short stop
	stepper_2.step(0, 1000);  // move motor 1000 steps in the other dirction
	delay(1000);            //short stop
	}
	else if(menuOption == '3') {
	// Stepper Motor with EasyDriver #3 - Test Code
	// the higher the time value the slower the motor will run
	stepper_3.step(1, 1000);  // move motor 1000 steps in one direction
	delay(1000);            // short stop
	stepper_3.step(0, 1000);  // move motor 1000 steps in the other dirction
	delay(1000);            //short stop
	}

	if (millis() - time0 > timeout)
	{
		menuOption = menu();
	}

}



// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions
char menu()
{

	Serial.println(F("\nWhich component would you like to test?"));
	Serial.println(F("(1) Stepper Motor with EasyDriver #1"));
	Serial.println(F("(2) Stepper Motor with EasyDriver #2"));
	Serial.println(F("(3) Stepper Motor with EasyDriver #3"));
	Serial.println(F("(menu) send anything else or press on board reset button\n"));
	while (!Serial.available());

	// Read data from serial monitor if received
	while (Serial.available())
	{
		char c = Serial.read();
		if (isAlphaNumeric(c))
		{

			if(c == '1')
				Serial.println(F("Now Testing Stepper Motor with EasyDriver #1"));
			else if(c == '2')
				Serial.println(F("Now Testing Stepper Motor with EasyDriver #2"));
			else if(c == '3')
				Serial.println(F("Now Testing Stepper Motor with EasyDriver #3"));
			else
			{
				Serial.println(F("illegal input!"));
				return 0;
			}
			time0 = millis();
			return c;
		}
	}
}
