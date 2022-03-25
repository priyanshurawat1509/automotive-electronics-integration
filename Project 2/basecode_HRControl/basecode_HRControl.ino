#include <PID_v1.h>
#include <Servo.h> //define the servo library

#define trigPin1 13       //trig sensor 1 to Arduino pin 13                                        
#define echoPin1 12       //Echo sensor 1 to Arduino pin 12
#define trigPin2 9       //trig sensor 2 to Arduino pin 11 
#define echoPin2 8       //Echo sensor 2 to Arduino pin 10 

Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc

int steering=90,throttle=90; //defining global variables to use later

//PID Constants
double Kp = 0, Ki = 10, Kd = 0;
double Input, Output, Setpoint;

PID my_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  
  Serial.begin(9600); //start serial connection. Uncomment for PC
  pinMode(trigPin1, OUTPUT);    // Set the trigPin1 as output
  pinMode(echoPin1, INPUT);     // Set the echoPin1 as input
  pinMode(trigPin2, OUTPUT);    // Set the trigPin2 as output
  pinMode(echoPin2, INPUT);     // Set the echoPin2 as input
  ssm.attach(10);               //define that ssm is connected at pin 10
  esc.attach(11);               //define that esc is connected at pin 11

  Setpoint = 0;
  my_PID.SetMode(AUTOMATIC);
  my_PID.SetTunings(Kp, Ki, Kd);
}

void loop() {

float duration1,duration2, distance1,distance2, error;

steering = 90;
throttle = 95;

  //First data point acquisition from the ultrasonic sensor 1 left
  
digitalWrite(trigPin1, HIGH);
delayMicroseconds(8);
digitalWrite(trigPin1, LOW);

duration1 = pulseIn(echoPin1, HIGH);
  
delay(2);

   //First data point acquisition from the ultrasonic sensor 2 right
  
digitalWrite(trigPin2, HIGH);
delayMicroseconds(8);
digitalWrite(trigPin2, LOW);

duration2 = pulseIn(echoPin2, HIGH);  

distance1 = (duration1 * 0.3435/ 2);   //convert into distance
distance2 = (duration2 * 0.3435/ 2);   //convert into distance

Input = distance1-distance2;

my_PID.Compute();

if(Input>=5){
  steering -= 1;
  setVehicleHR(steering, 95);
  delay(15);
}

if(Input<=-5){
  steering += 1;
  setVehicleHR(steering, 95);
  delay(15);
}

else(-5<Input<5 || throttle<100);
  throttle = throttle+5;
  setVehicleHR(steering, throttle);
  delay(15);
 

Serial.print(Output);
Serial.print(",\t");
delay(500);
}

//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//

void setVehicleHR(int s, int v) 
{
  s=min(max(0,s),180);  //saturate steering command
  v=min(max(70,v),110); //saturate velocity command
  ssm.write(s); //write steering value to steering servo
  esc.write(v); //write velocity value to the ESC unit
}

//***************** Do not change above part *****************//
