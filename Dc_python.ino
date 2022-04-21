#include "CmdMessenger.h"
enum {
    Dc1,
    Dc2,
    Dc1_value_is,
    Dc2_value_is,
    Dc1_direction_is,
    Dc2_direction_is,
};
#define in1 7
#define in2 8
#define in3 11
#define in4 12
byte get_Dcvar = 1 ;
const int BAUD_RATE = 9600;
CmdMessenger c = CmdMessenger(Serial,',',';','/');
int i1,i2,i3,i4;
void DCmotor(int i1,int i2,int i3,int i4){
  if(i1==1 && i3==1){
    digitalWrite(in1, i1);
    digitalWrite(in2, i2);
    digitalWrite(in3, i3);
    digitalWrite(in4, i4);
    delay(150);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  else if(i1==0 && i3==0){
    digitalWrite(in1, i1);
    digitalWrite(in2, i2);
    digitalWrite(in3, i3);
    digitalWrite(in4, i4);
    delay(150);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  else{
    digitalWrite(in1, i1);
    digitalWrite(in2, i2);
    digitalWrite(in3, i3);
    digitalWrite(in4, i4);
    delay(500);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
}
void getDC(int l,int h){
  if ( get_Dcvar == 1 ){
     i1 = l;
     i2 = h;
  }
  if ( get_Dcvar == 2 ){
     i3=l;
     i4=h;
  }
  ++get_Dcvar;
  if ( get_Dcvar >= 3 ){
    get_Dcvar=1 ;
    DCmotor(i1,i2,i3,i4);
  }
}
void on_Dc1(void){
  int d1=c.readBinArg<int>();
  if (d1 ==1){
    int i1=0;
    int i2=1;
    getDC(i1,i2);
    char val="FORWARD";
    c.sendBinCmd(Dc1_value_is,d1);
  }
  else if(d1==-1){
    int i1=1;
    int i2=0;
    getDC(i1,i2);
    char val="BACKWARD";
    c.sendBinCmd(Dc1_value_is,d1);
  }
  else if(d1==11){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    char val="FORWARD";
    c.sendBinCmd(Dc1_value_is,d1);
  }
  else if(d1==-11){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    char val="BACKWARD";
    c.sendBinCmd(Dc1_value_is,d1);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    char val="stopped";
    c.sendBinCmd(Dc1_value_is,d1);
  }
}

void on_Dc2(void){
  int d2=c.readBinArg<int>();
  if (d2 ==1){
    int i3=0;
    int i4=1;
    getDC(i3,i4);
    char val="FORWARD";
    c.sendBinCmd(Dc2_value_is,d2);
  }
  else if(d2==-1){
    int i3=1;
    int i4=0;  
    getDC(i3,i4);
    char val="BACKWARD";
    c.sendBinCmd(Dc2_value_is,d2);
  }
  else if(d2==11){
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    char val="FORWARD";
    c.sendBinCmd(Dc2_value_is,d2);
  }
  else if(d2==-11){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    char val="BACKWARD";
    c.sendBinCmd(Dc2_value_is,d2);
  }
  else{
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    char val="stopped";
    c.sendBinCmd(Dc2_value_is,d2);
  }
}

void attach_callbacks(void) { 
    c.attach(Dc1,on_Dc1);
    c.attach(Dc2,on_Dc2);
    }
void setup() {
    Serial.begin(BAUD_RATE);
    attach_callbacks();  
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    }


void loop() {
     //step (false, X_DIR, X_STP, 8*200);
    c.feedinSerialData();
}
