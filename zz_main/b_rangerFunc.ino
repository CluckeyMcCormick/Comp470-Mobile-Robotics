// Grove Ultrasonic Functions and Constants
// Specific to my configuration
// Nicolas Fredrickson 

#define pingPin A0

#define RANGER_LEFT     ZERO_PI
#define RANGER_FORWARD  PI_O2
#define RANGER_RIGHT    ONE_PI

/*
  ----------Ultrasonic Control Functions ----------------
*/
int pingCM(){
  long duration;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  delay(100);

  #ifdef DEBUG
  Serial.print("Distance Read: ");
  Serial.print(duration);
  Serial.print(" ---> ");
  Serial.print( (duration / 29) / 2 );
  Serial.println(" cm");
  #endif
  return (duration / 29) / 2;
}

int pingMM(){
  return pingCM() * 10;
}
