#include <Servo.h> //define the servo library

Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc

int steering=90,throttle=90; //defining global variables to use later

void setup() {
  //Serial.begin(9600); //start serial connection. Uncomment for PC
  ssm.attach(10);    //define that ssm is connected at pin 10
  esc.attach(11);     //define that esc is connected at pin 11
}

void loop() {
// Add control logic here

// Call the setVehicle function to set the vehicle steering and velocity(speed) values 

setVehicle(steering, throttle);

setVehicle (45, 90);
delay(3000);
setVehicle (90, 90);
delay(3000);
setVehicle (135, 95);
delay(3000);
setVehicle (90, 95);
delay(3000);

}

//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//
void setVehicle(int s, int v) 
{
  s=min(max(0,s),180);  //saturate steering command
  v=min(max(70,v),110); //saturate velocity command
  ssm.write(s); //write steering value to steering servo
  esc.write(v); //write velocity value to the ESC unit
}
//***************** Do not change above part *****************//
