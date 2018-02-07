/*
 ADXL3xx

 Reads an Analog Devices ADXL3xx accelerometer and communicates the
 acceleration to the computer.  The pins used are designed to be easily
 compatible with the breakout boards from Sparkfun, available from:
 http://www.sparkfun.com/commerce/categories.php?c=80

 http://www.arduino.cc/en/Tutorial/ADXL3xx

 The circuit:
 analog 0: accelerometer self test
 analog 1: z-axis
 analog 2: y-axis
 analog 3: x-axis
 analog 4: ground
 analog 5: vcc

 created 2 Jul 2008
 by David A. Mellis
 modified 30 Aug 2011
 by Tom Igoe

 This example code is in the public domain.

*/


#include <Servo.h>

Servo myservo_a;  // create servo object to control a servo
Servo myservo_b;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

// these constants describe the pins. They won't change:
const int groundpin = 18;             // analog input pin 4 -- ground
const int powerpin = 19;              // analog input pin 5 -- voltage
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)

const int base = 265;
const int maxVal = 405;
const int maxAngle = 180;

void setup() {
  // initialize the serial communications:
  Serial.begin(9600);

  // Provide ground and power by using the analog inputs as normal
  // digital pins.  This makes it possible to directly connect the
  // breakout board to the Arduino.  If you use the normal 5V and
  // GND pins on the Arduino, you can remove these lines.
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);

  myservo_a.attach(2);  // attaches the servo on pin 9 to the servo object
  myservo_b.attach(3);  // attaches the servo on pin 9 to the servo object
  perfectSet(0);
}

void loop() {
  // print the sensor values:
  Serial.println(analogRead(xpin));
  perfectSet( convert( analogRead(xpin) ) );
  delay(100);
}


void perfectSet(int angle){
    myservo_a.write(180 - angle);
    myservo_b.write(angle);  
}

void perfectSweep(){
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      perfectSet(pos);
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      perfectSet(pos);
      delay(15);                       // waits 15ms for the servo to reach the position
    }
}

int convert(int accelVal){
  int newValue = accelVal - base;
  newValue = newValue * (180 - 0);
  newValue = newValue / (maxVal - base);
  return newValue;
}

