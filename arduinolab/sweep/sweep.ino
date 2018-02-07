/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo_a;  // create servo object to control a servo
Servo myservo_b;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo_a.attach(2);  // attaches the servo on pin 9 to the servo object
  myservo_b.attach(3);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  //dualSet(180, 0);
  //positionCheck();
  perfectSweep();
}

void sweep(){
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo_a.write(pos);
      myservo_b.write(pos);// tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo_a.write(pos);
      myservo_b.write(pos);
      delay(15);                       // waits 15ms for the servo to reach the position
    }
}

void sprinkler(){
  for(pos = 0; pos <= 180; pos+= 5){
    myservo_a.write(pos);
    myservo_b.write(pos);
    delay(100);  
  }

  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo_a.write(pos);
      myservo_b.write(pos);             // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void stutter(){
  for(pos = 5; pos <= 180; pos+= 5){
    myservo_a.write(pos);
    myservo_b.write(pos);
    delay(100);
    myservo_a.write(pos - 5);
    myservo_b.write(pos - 5);
    delay(100); 
  }

  for (pos = 180 - 5; pos > 0; pos -= 5) { // goes from 180 degrees to 0 degrees
    myservo_a.write(pos);
    myservo_b.write(pos);
    delay(100);
    myservo_a.write(pos + 5);
    myservo_b.write(pos + 5);
    delay(100);                       
  }
}

void reset(int angle){
    myservo_a.write(angle);
    myservo_b.write(angle);
}

void microstep(int pos){
  myservo_a.write(pos);
  myservo_b.write(pos);
  delay(15);
  myservo_a.write(-pos);
  myservo_b.write(-pos);
  delay(15);
}

void circular(){
  for (pos = 0; pos <= 360; pos += 1) { // goes from 180 degrees to 0 degrees
      myservo_a.write(pos);
      myservo_b.write(pos);
      delay(100);                   
  }
}

void positionCheck(){
  reset(0);   delay(500); 
  reset(45);  delay(500); 
  reset(90);  delay(500); 
  reset(135); delay(500); 
  reset(180); delay(500); 
  reset(135); delay(500); 
  reset(90);  delay(500); 
  reset(45);  delay(500); 
  reset(0);   delay(500); 
}

void dualSet(int aAngle, int bAngle){
    myservo_a.write(aAngle);
    myservo_b.write(bAngle);  
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

