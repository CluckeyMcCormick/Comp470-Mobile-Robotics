// Diode (Light Detector) Functions and Constants
// Specific to my configuration
// Nicolas Fredrickson 

#define PIN_OUTPUT A2

enum {LEFT, RIGHT} lightState;

void diodeInit(){
  digitalWrite( PIN_OUTPUT, INPUT_PULLUP);
}

int getDiodeVal(){
  return analogRead( PIN_OUTPUT );
}

