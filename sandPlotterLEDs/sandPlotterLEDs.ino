#include <FastLED.h>

#define NUM_LEDS 71
#define DATA_PIN 13

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS); // Use GRB or RGB as needed
  FastLED.clear();
  FastLED.show();
}

void loop() {
  // Set all LEDs to red
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(500);

  // Set all LEDs to green
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(500);

  // Set all LEDs to blue
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(500);
}
