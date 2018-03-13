#include <SPI.h>
#include <Adafruit_DotStar.h>

// ========== DARK-FIELD ILLUMINATOR LIGHTS (ADAFRUIT DOTSTAR LEDs) ========== //

#define NUM_DOTSTAR_LEDS 144        // Number of LEDs we are driving

// create a list of colors for the lights
// we are basically creating an HSV gradient here
#define NUM_DOTSTAR_COLORS 8
// list of colors (solid colors fading to dark)
const uint32_t dotstarColorList[NUM_DOTSTAR_COLORS] = {0xFFFFFF, 0xDFDFDF, 0xBFBFBF, 0x9F9F9F, 0x7F7F7F, 0x5F5F5F, 0x3F3F3F, 0x1F1F1F};
//const uint32_t dotstarColorList[NUM_DOTSTAR_COLORS] = {0xFF0000, 0xE50000, 0xCC0000, 0xB20000, 0x990000, 0x7F0000, 0x660000, 0x4C0000};
// Here's how to control the LEDs from any two pins:
// The below code is for software SPI on pins 8 & 9
//#define DATAPIN    8
//#define CLOCKPIN   9
/*Adafruit_DotStar strip = Adafruit_DotStar(
  NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);*/
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, which apprently doesn't work with the latest
// production runs. DOTSTAR_BGR worked for me.

// Hardware SPI is a little faster, but must be wired to specific pins (and that's what we're using!)
// (Arduino Uno & Pro/Pro Mini = pin 11 for data, 13 for clock, Mega 2560 = pin 51 for data, 52 for clock; other boards may be different).
Adafruit_DotStar ledStrip = Adafruit_DotStar(NUM_DOTSTAR_LEDS, DOTSTAR_BGR);

void setDotstarLEDColors(uint8_t colorIndex, uint8_t brightness) {
  // set the brightness
  ledStrip.setBrightness(brightness);

  uint32_t color;
  // select the correct color from the array
  if(colorIndex >= NUM_DOTSTAR_COLORS) {
    color = dotstarColorList[NUM_DOTSTAR_COLORS - 1]; // making sure we don't run over the array bounds
  } else {
    color = dotstarColorList[colorIndex];
  }

  // set the color for all the pixels/leds
  for (uint8_t i = 0; i < NUM_DOTSTAR_LEDS; ++i) {
    ledStrip.setPixelColor(i, color);
  }
  // show the updated pixels
  ledStrip.show();
}


void setup() {
  // Dotstar LEDs for dark-field illuminator
  ledStrip.begin();                  // Initialize LED pins for output
  ledStrip.clear();                  // Set all pixel data to zero
  ledStrip.show();                   // Turn all LEDs off ASAP
  // initiaize dotstars at full white/full brightness
  setDotstarLEDColors(0, 255);
}

void loop() {
//  // scrol thru all the colors
  setDotstarLEDColors(0, 255);
//  delay(5000);
//  setDotstarLEDColors(1, 255);
//  delay(5000);
//  setDotstarLEDColors(2, 255);
//  delay(5000);
//  setDotstarLEDColors(3, 255);
//  delay(5000);
//  setDotstarLEDColors(4, 255);
//  delay(5000);
//  setDotstarLEDColors(5, 255);
//  delay(5000);
//  setDotstarLEDColors(6, 255);
//    delay(5000);
//    setDotstarLEDColors(7, 255);
//    delay(5000);

}


