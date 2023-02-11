#define trigPin 11
#define echoPin 10
int LED = 4;
int beep = 5;

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("Start");
}

void loop() {
  float duration, distance, timespan, Y, Yi, K, P, Pi, z, R, covariance_limit, trig_delay, loop_delay, Distance_Final;

  trig_delay = 8;             //*******************************Change
  loop_delay = 10;            //*******************************Change

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(trig_delay);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  Serial.print(duration, 4);
  Serial.print("\n");

  Yi = duration;
  Pi = 200;
  //R = duration * 3.65801157014395 - 67.6450540109426; //*************************************Change
  //R=(pow(duration,3)*(-2.56891451700051e-06)) +(pow(duration,2)*(0.00646870073593194))+((duration)*(-4.18166080143902)) +(739.754102994190);
  
//R= (pow(duration,2)*(-0.000118390442703529))+((duration)*(0.547263327001572)) -(90.7568018714627);
  R=55;
  
  covariance_limit = duration * 0.462784615384615  - 89.1769230769229; //*************************************Change
//-0.000118390442703529  0.547263327001572 -90.7568018714627
  for (int i = 0; i <= 1000; i++) {

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(trig_delay);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);

    z = duration;
    P = Pi;
    Y = Yi;

    K = P / (P + R);
    Yi = Y + K * (z - Y);
    Pi = (1 - K) * P;

  
    //z =(duration/2)*0.3435*(1500/1476.5); //0.000117600825741831 * (pow(duration, 2))  - 0.0316696980069243 * (pow(duration, 1)) + 59.8725150448621555 * (duration)  - 23.8732402545406;
    z=(duration* 0.174413473086394 )+(1.57717868662126);//(16.1411357746832);
  
    //Distance_Final  =(Yi/2)*0.3435*(1500/1476.5); 
    //Distance_Final=0.000117600825741831 * (pow(Yi, 2))  - 0.0316696980069243 * (pow(Yi, 1)) + 59.8725150448621555 * (Yi)  - 23.8732402545406; //*************************************Change
Distance_Final=(Yi* 0.174413473086394 )+(1.57717868662126);
    //Distance_Final=(Yi* 0.174413473086394 )+(1.57717868662126);//(16.1411357746832);
    Serial.print(Yi, 4);
    Serial.print("\t");
    Serial.print(z, 4);
    Serial.print("\t");
    Serial.print(Pi, 4);
    Serial.print("\t");
    Serial.print(Distance_Final, 4);
    Serial.print("\t");
    Serial.print("\n");

    delay(loop_delay);

    if (Pi < 0.001) {
      timespan = millis();
      Serial.print("\n");
      Serial.print("end");
      Serial.print("\n");
      Serial.print(timespan);
      digitalWrite(LED, HIGH);
      digitalWrite(beep, HIGH);
      delay(1000);
      digitalWrite(LED, LOW);
      digitalWrite(beep, LOW);

      Serial.end();
    }
  }
  Serial.end();
}
