#define trigPin 13
#define echoPin 12

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  float duration, distance, timespan, Y, Yi, K, P, Pi, z, R;

  //digitalWrite(trigPin, LOW);
  //delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.03435;

  Serial.print(distance);
  Serial.print("\n");
  Serial.print("END");
  delay(2);

  Yi = distance;
  Pi = distance;
  R = 0.02027236;

  for (int i = 0; i <= 10; i++) {


    //digitalWrite(trigPin, LOW);
    //delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(3);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2) * 0.03435;

    z = distance;
    Y = Yi;
    P = Pi;

    K = P / (P + R);
    Yi = Y + K * (z - Y);
    Pi = (1 - K) * P;

    Serial.print(Yi);
    Serial.print("\n");
    Serial.print(Pi);
    Serial.print("\t");
    delay(2);
    if (Yi == Y) {
      timespan = millis();
      Serial.print("\n");
      Serial.print("\n");
      Serial.print(timespan);
      Serial.end();
    }
  }



}
