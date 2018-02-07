const int ledPin = 13;
const int anaPin = 0; 

int val = 0;

void setup() {
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  int val = analogRead(anaPin);
  if(val >= 512)
    digitalWrite(ledPin, HIGH);
  else
    digitalWrite(ledPin, LOW);
    
  Serial.print("Value:");
  Serial.print(val);
  Serial.print("\n");
}
