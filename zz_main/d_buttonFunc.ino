// Button Functions and Constants
// Specific to my configuration
// Nicolas Fredrickson 

#define buttonPin 11

//Initializer Function
void buttonInit(){
  pinMode(buttonPin, INPUT);
}

/*
  -------------- Button Functions ----------------
*/

inline void checkButtonState(){
  if( digitalRead(buttonPin) )
    moveState = STOPPED;
}

