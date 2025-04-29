#include <FastLED.h>

#define LED_PIN     2
#define COLOR_ORDER RGB
#define CHIPSET     WS2811
#define NUM_LEDS    250

#define BRIGHTNESS  255
//CRGB color  = CHSV(40*256/360,230,255); // pure hue, three-quarters brightness

CRGB color = CRGB::White;
CRGB leds[NUM_LEDS];
//CRGBPalette16 palette = LavaColors_p;
CRGBPalette16 palette = CRGBPalette16(CRGB::White, CRGB::Red);


void setup() {
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( BRIGHTNESS );
}

void loop() {
  uint32_t now = millis();
  uint32_t offset = now * 256 / 3000;
  for (int i = 0; i < NUM_LEDS; i++) {
    //if((now / 100)%2) {
      leds[i] = ColorFromPalette(palette, (i * 256 * 1 / NUM_LEDS + offset) % 256);
    //} else {
    //  leds[i] = CRGB::Black;
    //}
  }
  FastLED.show();

}
