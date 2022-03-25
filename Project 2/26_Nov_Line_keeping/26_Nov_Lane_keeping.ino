#include <Servo.h> //define the servo library

#define trigPin1 13      //trig sensor 1 to Arduino pin 13                                        
#define echoPin1 12      //Echo sensor 1 to Arduino pin 12
#define trigPin2 9       //trig sensor 2 to Arduino pin 9
#define echoPin2 8       //Echo sensor 2 to Arduino pin 8
#define trigPin3 7       //trig sensor 2 to Arduino pin 7 
#define echoPin3 6       //Echo sensor 2 to Arduino pin 6 

//PID sides parameters

#define kp 5
#define ki 0
#define kd 0

double priError = 0;
double toError = 0;

//PID front parameters

#define kp_front 10
#define ki_front 0
#define kd_front 0.1

double priError_front = 0;
double toError_front = 0;

Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc

int steering = 90, throttle = 90; //defining global variables to use later

void setup() {

  Serial.begin(9600); //start serial connection. Uncomment for PC
  pinMode(trigPin1, OUTPUT);    // Set the trigPin1 as output
  pinMode(echoPin1, INPUT);     // Set the echoPin1 as input
  pinMode(trigPin2, OUTPUT);    // Set the trigPin2 as output
  pinMode(echoPin2, INPUT);     // Set the echoPin2 as input
  pinMode(trigPin3, OUTPUT);    // Set the trigPin3 as output
  pinMode(echoPin3, INPUT);     // Set the echoPin3 as input
  ssm.attach(10);               //define that ssm is connected at pin 10
  esc.attach(11);               //define that esc is connected at pin 11
}

void loop() {

  PID();

}
//********************** Vehicle Control **********************//

void PID() {
  float duration1, duration2, duration3, distance1, distance2, distance3, delta_distance, LR_multiplier;

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

  delay(2);
  
  //First data point acquisition from the ultrasonic sensor 3 front

  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(8);
  digitalWrite(trigPin3, LOW);

  duration3 = pulseIn(echoPin3, HIGH);

  distance1 = (duration1 * 0.03435 / 2);  //convert into distance
  distance2 = (duration2 * 0.03435 / 2);  //convert into distance
  distance3 = (duration3 * 0.03435 / 2);  //convert into distance

  distance1 = min(distance1, 38); //left
  distance2 = min(distance2, 38); //right
  //distance3 = min(distance3, 40); //front

//  Serial.print(distance1, 2); //print distance data of sensor 1 (left)
//  Serial.print("\t");
//  Serial.print(distance2, 2); //print distance data of sensor 2 (right)
//  Serial.print("\t");
  Serial.print(distance3, 2); //print distance data of sensor 3 (front)
  Serial.print("\t");

  if (distance1 > distance2) {
    LR_multiplier = 1;
  }
  else {
    LR_multiplier = -1;
  }

  delta_distance = min(distance1, distance2); //min of left and right

//  Serial.print(delta_distance, 2);
//  Serial.print("\t");

  int setP = 38;        //required side distance
  double error = setP-delta_distance;   //error side

  int setP_front = 38;        //required front distance
  double error_front = distance3-setP_front ;   //error side

////////////////////THROTTLE////////////////////////////

  double Pvalue_front = error_front * kp_front;
  double Ivalue_front = toError_front * ki_front;
  double Dvalue_front = (error_front - priError_front) * kd_front;
    
  double PIDvalue_front = Pvalue_front + Ivalue_front + Dvalue_front;
  priError_front = error_front;
  toError_front += error_front;

  int Fvalue_front = (int)PIDvalue_front;
  Serial.print(Fvalue_front);
  Serial.print("\t");
  
  Fvalue_front = map(Fvalue_front, -100, 100, 70, 110);

if (Fvalue_front < 80) {
    Fvalue_front = 80;
  }
  if (Fvalue_front > 95) {
    Fvalue_front = 95;
  }
  
  Serial.print(Fvalue_front);
  Serial.print("\t");
///////////////////////////////////////////////////////////


/////////////////////STEERING/////////////////////////
  double Pvalue = error * kp;
  double Ivalue = toError * ki;
  double Dvalue = (error - priError) * kd;

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  priError = error;
  toError += error;

  int Fvalue = (int)PIDvalue;

//  Serial.print(Fvalue); Serial.print("\t");

  Fvalue = Fvalue * LR_multiplier;
  Fvalue = map(Fvalue, -180, 180, 190, 10);

//  Serial.print(Fvalue); Serial.print("\t");

  if (Fvalue < 10) {
    Fvalue = 10;
  }
  if (Fvalue > 190) {
    Fvalue = 190;
  }

  setVehicleHR(steering, throttle);
  steering = Fvalue;
  throttle = Fvalue_front;
  setVehicleHR(Fvalue, Fvalue_front);
  Serial.print(steering);
  Serial.print("\t");
  Serial.print(throttle);
  Serial.print("\n");
//////////////////////////////////////////////////////////
delayMicroseconds(650);
setVehicleHR(steering, throttle);
steering = Fvalue;
throttle = 90;
setVehicleHR(Fvalue, Fvalue_front);

}


//***************** Do not change below part *****************//

void setVehicleHR(int s, int v)
{
  s = min(max(0, s), 180); //saturate steering command
  v = min(max(70, v), 110); //saturate velocity command
  ssm.write(s);         //write steering value to steering servo
  esc.write(v);         //write velocity value to the ESC unit
}

//***************** Do not change above part *****************//
