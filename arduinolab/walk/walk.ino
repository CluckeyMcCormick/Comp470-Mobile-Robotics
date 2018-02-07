/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo critter_right;  // create servo object to control a servo
Servo critter_left;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  critter_right.attach(2);  // attaches the servo on pin 9 to the servo object
  critter_left.attach(4);  // attaches the servo on pin 9 to the servo object

  reset();
}

void loop() {
  reset();
}

void walk(){
    for (pos = 0; pos <= 180; pos += 1) {
      // in steps of 1 degree
      critter_right.write(pos);
      delay(15);                       
    }

    delay(1000);
    
    for (pos = 180; pos >= 0; pos -= 1) { 
      critter_left.write(pos);
      delay(15);                      
    }

    reset();
}

void walk2(){
    for (pos = 0; pos <= 180; pos += 1) {
      // in steps of 1 degree
      critter_right.write(180 - pos);
      critter_left.write(180 - pos);
      delay(15);                       
    }

    reset();
}

void reset() {
    critter_right.write(0);
    critter_left.write(180);
    delay(100);  
}

