#include <FastLED.h>

#define PIN_LEDS 2

#define NUM_LEDS_TOTAL 47


CRGB leds[NUM_LEDS_TOTAL];
static const CRGBPalette16 palette(CRGB::Purple, CRGB::Purple, CRGB::Purple, CRGB::Purple,
                                   CRGB::Purple, CRGB::Purple, CRGB::Purple, CRGB::White,
                                   CRGB::Purple, CRGB::Purple, CRGB::Purple, CRGB::White,
                                   CRGB::Purple, CRGB::Purple, CRGB::Purple, CRGB::White);
//RainbowColors_p;

void setup() {
  // Setup the LED strip
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS_TOTAL);
  FastLED.setBrightness(20);
}

void loop() {
  int32_t t = millis() / 3;
  for (int i = 0; i < NUM_LEDS_TOTAL; i++) {
    leds[NUM_LEDS_TOTAL - i - 1] = ColorFromPalette(palette, (i * 255 / NUM_LEDS_TOTAL + t) % 255, 255, LINEARBLEND);
  }
  FastLED.show();
}
