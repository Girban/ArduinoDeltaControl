 /* -----------------------------------------------------------------------------
 * Example .ino file for arduino, compiled with CmdMessenger.h and
 *
 * CmdMessenger.cpp in the sketch directory.
 *----------------------------------------------------------------------------*/
#include "CmdMessenger.h"
//#include <Stepper.h>

/* Define available CmdMessenger commands */
enum {
	motor1,
	motor2,
	motor3,
	motor1_value_is,
	motor2_value_is,
	motor3_value_is,
	motor1_direction_is,
	motor2_direction_is,
	motor3_direction_is,
	gripper,
	pump_is,
};

# define EN 8 // stepper motor enable , active low
# define X_DIR 5 // X -axis stepper motor direction control
# define Y_DIR 6 // y -axis stepper motor direction control
# define Z_DIR 7 // z axis stepper motor direction control
# define X_STP 2 // x -axis stepper control
# define Y_STP 3 // y -axis stepper control
# define Z_STP 4 // z -axis stepper control
const int pumpPin=28;
byte get_var = 1 ;
int dx,dy,dz ; byte dpx,dpy,dpz ; byte spx,spy,spz ; int sx,sy,sz ;
int state;
const int step_per_rev = 6400 ;
//Stepper steppermotor(step_per_rev ,X_STP,Y_STP,Z_STP);
int dist;
void step (int sx,int sy,int sz,int dx, int dy, int dz)
{
	int ST[]={sx,sy,sz};
	int sh=ST[0];
	for (int i=0;i<3;i++){
		if(sh<ST[i]){
			sh=ST[i];
		}
	}
	int delays[sh];
	float angle = 1;
	float accel = 0.01;
	float c0 = 5000 * sqrt( 2 * angle / accel ) * 0.67703;
	float lastDelay = 0;
	int highSpeed = 100;
	for (int i = 0; i < sh; i++) {
		float d = c0;
		if ( i > 0 )
			d = lastDelay - (2 * lastDelay) / (4 * i + 1);
		if ( d < highSpeed )
			d = highSpeed;
		delays[i] = d;
		lastDelay = d;
	}
		//digitalWrite(EN,HIGH);
		digitalWrite (dpx, dx);
		if(dy==1){
			digitalWrite (dpy, 1);
		}
		else{
			digitalWrite (dpy, 0);
			}
		if(dz==1){
			digitalWrite (dpz, 1);
		}
		else{
			digitalWrite (dpz, 0);
		}

		//delay (50);
		for (int i = 0; i < sh ; i++)
		{
			 if ( i < sx ){
				digitalWrite (spx, HIGH);
			 }
			 if ( i < sy ){
				digitalWrite (spy, HIGH);
			 }
			 if ( i < sz ){
				digitalWrite (spz, HIGH);
			 }

			 if ( i > sx && i > sy && i > sz )
				break;

				delayMicroseconds( delays[i] );  //going below 500 will increase motor speed  and vice versa .But there is a limit.Motor will not move properly at very high or very low delays.
				digitalWrite (spx, LOW);
				digitalWrite (spy, LOW);
				digitalWrite (spz, LOW);

		}
		for (int i = 0; i < sh ; i++)
		{
			 if ( i < sx ){
				digitalWrite (spx, HIGH);
			 }
			 if ( i < sy ){
				digitalWrite (spy, HIGH);
			 }
			 if ( i < sz ){
				digitalWrite (spz, HIGH);
			 }

			 if ( i > sx && i > sy && i > sz )
				break;

				delayMicroseconds( delays[sh-i-1] );  //going below 500 will increase motor speed  and vice versa .But there is a limit.Motor will not move properly at very high or very low delays.
				digitalWrite (spx, LOW);
				digitalWrite (spy, LOW);
				digitalWrite (spz, LOW);

		}

}

void get_data(boolean d, byte dp, byte sp, int s) {
	if ( get_var == 1 ){
		dx = d ;
		dpx = dp ;
		spx = sp ;
		sx = s ;
	}
	if ( get_var == 2 ){
		dy = d ;
		dpy = dp ;
		spy = sp ;
		sy = s ;
	}
	if ( get_var == 3 ){
		dz = d ;
		dpz = dp ;
		spz = sp ;
		sz = s ;
	}
	++get_var;
	if ( get_var >= 4 ){
		get_var=1 ;
		step(sx,sy,sz,dx,dy,dz);
	}
}


/* Initialize CmdMessenger -- this should match PyCmdMessenger instance */
const int BAUD_RATE = 9600;
CmdMessenger c = CmdMessenger(Serial,',',';','/');
int value1,value2,value3;
/* Create callback functions to deal with incoming messages */

/* callback */


void on_motor1(void){
		value1 = c.readBinArg<int>();
		int x=value1;
	 // c.sendBinCmd(motor1_value_is,value1);
		int dir1=0;
		if (value1<0)
				{dir1=1;
				x=-x;}
	// Serial.println(value1);
//   step (dir1, X_DIR, X_STP, x*16);
		get_data( dir1, X_DIR, X_STP, x);
		c.sendBinCmd(motor1_value_is,value1);
	 //delay(2000);
}
void on_motor2(void){
		 value2 = c.readBinArg<int>();
		 int y=value2;
	//  c.sendBinCmd(motor2_value_is,value2);
		 int  dir2=0;
		 if (value2<0)
				{dir2=1;
				y=-y;}
	// Serial.println(value1);
//   step (dir2, Y_DIR, Y_STP, y*8.96);
		 get_data( dir2, Y_DIR, Y_STP, y);
		 c.sendBinCmd(motor2_value_is,value2);
	 //delay(2000);
}
void on_motor3(void){
		value3 = c.readBinArg<int>();
		int z=value3;
		int dir3=0;
		if (value3<0)
				{dir3=1;
				z=-z;
				}
	 //Serial.println(value1);
//   step (dir3, Z_DIR, Z_STP, z*8.96);
		get_data( dir3, Z_DIR, Z_STP, z);
		c.sendBinCmd(motor3_value_is,value3);
	// delay(2000);
}

void on_gripper(void){
	dist=c.readBinArg<int>();
	if (dist ==1){
		digitalWrite(pumpPin, HIGH);
		delay(200);
		state=dist;
		c.sendBinCmd(pump_is,state);
	}
	else if(dist==0){
		digitalWrite(pumpPin, LOW);
		delay(200);
		state=dist;
		c.sendBinCmd(pump_is,state);
	}
}

/* Attach callbacks for CmdMessenger commands */
void attach_callbacks(void) {

		c.attach(motor1,on_motor1);
		c.attach(motor2,on_motor2);
		c.attach(motor3,on_motor3);
		c.attach(gripper,on_gripper);
}

void setup() {
		Serial.begin(BAUD_RATE);
		attach_callbacks();
		pinMode(pumpPin, OUTPUT);
		pinMode (X_DIR, OUTPUT); pinMode (X_STP, OUTPUT);
		pinMode (Y_DIR, OUTPUT); pinMode (Y_STP, OUTPUT);
		pinMode (Z_DIR, OUTPUT); pinMode (Z_STP, OUTPUT);
		pinMode (EN, OUTPUT);
		digitalWrite(EN, LOW);

}

void loop() {
		 //step (false, X_DIR, X_STP, 8*200);
		c.feedinSerialData();


	 //step (true, X_DIR, X_STP, 8*200);
	// step (false, Y_DIR, Y_STP, value2*(200/360)); // y axis motor reverse 1 ring, the 200 step is a circle.

//   step (false, Z_DIR, Z_STP,value3*(200/360)); // z axis motor reverse 1 ring, the 200 step is a circle.

}
