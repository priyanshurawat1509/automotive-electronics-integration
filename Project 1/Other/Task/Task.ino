#define trigPin 13
#define echoPin 12
int LED = 4;
int beep = 5;

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("Start");
  
}

void loop() {
  float duration, distance, timespan, Y, Yi, K, P, Pi, z, R, covariance_limit, trig_delay;

trig_delay = 5;             //*******************************Change

  //digitalWrite(trigPin, LOW);
  //delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(trig_delay);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.3435;
  distance = 1.55951305722164e-06*(pow(distance,2))  -0.000466457893072978*(pow(distance,1)) +1.02137694207455*(distance)  -1.3270719729626845;

  Serial.print(distance,4);
  Serial.print("\n");
  delay(2);

  Yi = distance;
  Pi = distance;
  R = distance*0.00110903209401274 -0.0127805363643312;//*************************************Change
  covariance_limit = distance*0.0000692307692307692 -0.00384615384615381;//*************************************Change
                    
  for (int i = 0; i <= 1000; i++) {

    //digitalWrite(trigPin, LOW);
    //delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(6);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2) * 0.3435;

    z = 1.55951305722164e-06*(pow(distance,2))  -0.000466457893072978*(pow(distance,1)) +1.02137694207455*(distance)  -1.3270719729626845;//*************************************Change
    Y = Yi;
    P = Pi;

    K = P / (P + R);
    Yi = Y + K * (z - Y);
    Pi = (1 - K) * P; 

    Serial.print(distance,4);
    Serial.print("\t");
    Serial.print(Pi,4);
    Serial.print("\t");
    Serial.print(Yi,4);
    Serial.print("\n");
   
    delay(8);
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
