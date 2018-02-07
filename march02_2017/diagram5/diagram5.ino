const int ledPin = 13;
const int inputPin = 2; 

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(inputPin, INPUT);
}

void loop() {
  int val = digitalRead(inputPin);
  if(val == HIGH)
    digitalWrite(ledPin, HIGH);
  else
    digitalWrite(ledPin, LOW);
}
