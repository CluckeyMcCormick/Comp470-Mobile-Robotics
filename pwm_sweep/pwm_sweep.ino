#define SERVO_PIN         2  // 26 or 39. Any pin on the Arduino or Mega will work.

int lenMicroSecondsOfPeriod = 25 * 1000;       // 25 milliseconds (ms). found much better smoothness at 25, not 20 ms.
int lenMicroSecondsOfPulse = 0;                // used in the while loop below
int lenMicroSecondsOfPulseStart = 0.5 * 1000;  // 0 degrees
int lenMicroSecondsOfPulseEnd = 2.2 * 1000;    // 180 degrees
int lenMicroSecondsOfPulseStep = 0.01 * 1000;   // .1 millisecond. That's 200 increments b/w 1.0 and 2.0

void setup()
{
  pinMode(SERVO_PIN, OUTPUT);

  // Setup our start point for our main loop
  lenMicroSecondsOfPulse = lenMicroSecondsOfPulseStart + lenMicroSecondsOfPulseStep;

}

void loop()
{

 // Servos work by sending a 20 ms pulse.
 // 1.0 ms at the start of the pulse will turn the servo to the 0 degree position
 // 1.5 ms at the start of the pulse will turn the servo to the 90 degree position
 // 2.0 ms at the start of the pulse will turn the servo to the 180 degree position

 // Do a while loop starting at our start pulse and incrementing each time thru the loop
 // Stop when we reach our final end point
for(int i = 0; i < 200; i++)
 {
   // Turn voltage high to start the period and pulse
   digitalWrite(SERVO_PIN, HIGH);

   // Delay for the length of the pulse
   delayMicroseconds(lenMicroSecondsOfPulse);

   // Turn the voltage low for the remainder of the pulse
   digitalWrite(SERVO_PIN, LOW);

   // Delay this loop for the remainder of the period so we don't
   // send the next signal too soon or too late
   delayMicroseconds(lenMicroSecondsOfPeriod - lenMicroSecondsOfPulse); 

   // Increment our pulse
   lenMicroSecondsOfPulse += lenMicroSecondsOfPulseStep;

 }

 // Now reverse the step so we go in the opposite direction
 lenMicroSecondsOfPulseStep = lenMicroSecondsOfPulseStep * -1;
 lenMicroSecondsOfPulse += lenMicroSecondsOfPulseStep;

 // delay for a few seconds and do it all again
 delay(2 * 1000);

}
