// RGB Light Functions and Constants
// Specific to my configuration
// Nicolas Fredrickson

#include <ChainableLED.h>

#define NUM_LEDS    1

#define PIN_CLOCK   SCL
#define PIN_DATA    SDA

#define DEFAULT_SATURATION  1.0
#define DEFAULT_BRIGHTNESS  0.15

#define COLOR_HUE_ONE   0.0
#define COLOR_HUE_TWO   0.25
#define COLOR_HUE_THREE 0.5
#define COLOR_HUE_FOUR  0.75

//defines the pin used on arduino.
ChainableLED light(PIN_CLOCK, PIN_DATA, NUM_LEDS);

void inline lightInit(){
  light.init();
}

/*
  -------------- Light Functions ----------------
*/
inline void radianToColor(float radianAngle){
  light.setColorHSB(0, fmod(radianAngle, TWO_PI) / TWO_PI, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS);
}

inline void degreeToColor(int degree){
  light.setColorHSB(0, (degree % 360) / 360, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS);
}

inline void radianToSat(float radianAngle, double color){
  light.setColorHSB(0, color, fmod(radianAngle, TWO_PI) / TWO_PI ,DEFAULT_BRIGHTNESS );
}

inline void radianToBright(float radianAngle, double color){
  light.setColorHSB(0, color, DEFAULT_SATURATION, fmod(radianAngle, TWO_PI) / TWO_PI );
}

inline void specificColor(double color){
  light.setColorHSB(0, color, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS );
}

inline void rgbColor(int red, int green, int blue){
  light.setColorRGB(0, red, green, blue);
}

inline void lightOff(){
  light.setColorHSB(0, 0, 0, 0);
}

inline void lightCycle(int delayTime){
  for (int i = 0; i < 10; i++)
    {
      rgbColor( 100, 0, 0 );  // red
      delay(delayTime);
      rgbColor( 0, 100, 0 );  // green
      delay(delayTime);
      rgbColor( 0, 0, 100 );  // blue
      delay(delayTime);
    }
}
