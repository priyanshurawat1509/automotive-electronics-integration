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

#define kp_front 0.7
#define ki_front 0.0001
#define kd_front 0.2

double K1 = kp_front + ki_front + kd_front;
double K2 = kp_front - 2 * kd_front;
double K3 = kd_front;

double E1, E2, E = 0;

double priError_front = 0;
double toError_front = 0;
int diff;
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
  float duration1, duration2, duration3, distance1, distance2, distance3, delta_distance, LR_multiplier, u, u1;

  int setP_front = 36;        //required front distance
  int setP = 38;        //required side distance

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

  Serial.print(distance3, 2); //print distance data of sensor 3 (front)
  Serial.print("\t");

  if (distance1 > distance2) {
    LR_multiplier = 1;
  }
  else {
    LR_multiplier = -1;
  }

  delta_distance = min(distance1, distance2); //min of left and right

  double error = setP - delta_distance; //error side

  ////////Error PID Front

  double E = distance3 - setP_front ; //error front
  E2 = E1;
  E1 = E;

  Serial.print(E, 2);
  Serial.print("\t");

  Serial.print(E1, 2);
  Serial.print("\t");
  Serial.print(E2, 2);
  Serial.print("\t");

  //if (E>50){
  //E = distance3-50 ;
  //}
  //if (E>50){
  //E = distance3-100 ;
  //}
  //Serial.print(E, 2);
  //Serial.print("\t");

  //////////////////THROTTLE////////////////////////////
  u1 = u;
  u = u1 + (K1 * E) + (K2 * E1) + (K3 * E2);

  Serial.print(u);
  Serial.print("\t");

  float Fvalue_front = u;
  int throttle_new = 90 + Fvalue_front;

  Serial.print(Fvalue_front);
  Serial.print("\t");

  Serial.print(throttle_new);
  Serial.print("\t");

  if (throttle_new < 85) {
    throttle_new = 85;
  }
  else if (throttle_new > 95) {
    throttle_new = 95;
  }

  Serial.print(throttle_new);
  Serial.print("\t");

  //////////////////Steering Backwards*//////////////////////

  int FR_multiplier = 1;
  if (throttle_new < 89.9999) {
    FR_multiplier = -1;
  }
  else if (throttle_new > 89.999) {
    FR_multiplier = 1;
  }

  /////////////////////STEERING/////////////////////////
  double Pvalue = error * kp;
  double Ivalue = toError * ki;
  double Dvalue = (error - priError) * kd;

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  priError = error;
  toError += error;

  int Fvalue = (int)PIDvalue;

  //  Serial.print(Fvalue); Serial.print("\t");

  Fvalue = Fvalue * LR_multiplier * FR_multiplier;
  Fvalue = map(Fvalue, -180, 180, 125, 70);

  if (Fvalue < 70) {
    Fvalue = 70;
  }
  if (Fvalue > 125) {
    Fvalue = 125;
  }

  setVehicleHR(steering, throttle);
  steering = Fvalue;
  throttle = throttle_new;
  throttle = max(min(throttle, 95), 85);
  diff = throttle - 90;
  if (abs(diff) < 5) {
    if (diff < 0)
      setVehicleHR(Fvalue, 85);
    else if (diff > 0)
      setVehicleHR(Fvalue, 95);
    delay(int(abs(diff) * 8));
    setVehicleHR(Fvalue, 90);
  }
  setVehicleHR(Fvalue, throttle_new);

  //////////////////////////////////////////////////////////

    delayMicroseconds(30);
    setVehicleHR(steering, 90);;
//    Serial.print("\n");
    delayMicroseconds(15000);

//    setVehicleHR(steering, 90);
    setVehicleHR(steering, throttle);

  ////////////////////////************************************************
//  if (FR_multiplier = 1) {
//    delayMicroseconds(10000);
//  }


  setVehicleHR(steering, throttle);

  //First data point acquisition from the ultrasonic sensor 3 front

  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(8);
  digitalWrite(trigPin3, LOW);

  float duration4 = pulseIn(echoPin3, HIGH);

  float distance4 = (duration4 * 0.03435 / 2);  //convert into distance

  if (duration4 < (0.995 * duration3) || duration4 > (1.005 * duration3)) { //Error Rejection
    setVehicleHR(steering, 90);
  }
  /////////////////////////////////////////////////////////////////////////////LAST

  Serial.print("\n");
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
