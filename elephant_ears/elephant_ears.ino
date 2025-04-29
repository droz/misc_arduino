#include <FastLED.h>
#include <TXOnlySerial.h>

#define PIN_LEDS 1
#define RX_PIN 0
#define TX_DEBUG_PIN 2
#define TIMER_DEBUG_PIN 4

#define NUM_LEDS_TOTAL 8

CRGB leds[NUM_LEDS_TOTAL];
//static const CRGBPalette16 palette = RainbowStripeColors_p;
//static const CRGBPalette16 palette = LavaColors_p;
//static const CRGBPalette16 palette = OceanColors_p;
static const CRGBPalette16 palette = ForestColors_p;
//static const CRGBPalette16 palette = CloudColors_p;

TXOnlySerial serial(TX_DEBUG_PIN);

void setup() {
  // Setup the LED strip
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS_TOTAL);
  FastLED.setBrightness(255);

  // Setup the Universal Serial Interface (USI)
  pinMode(RX_PIN, INPUT);
  pinMode(TIMER_DEBUG_PIN, OUTPUT);
  // Timer 0 configuration
  GTCCR = 1 << TSM; // Stop the timer while we are configuring
  OCR0A = 35; // 16Mhz / (35 * 4) = 115.1Mhz
  OCR0B = 1;
  TCCR0A = (3 << WGM00);  // Fast PWM mode
  TCCR0B = (1 << WGM02) |  // Fast PWM mode
           (1 << CS00);   // Use clock input directly (no prescaler)
  GTCCR = 0 << TSM; // Start the timer

  TCCR0A |= (2 << COM0B0); // Timer toggles output on port B

  // USI configuration register
  USICR = 0 << USISIE | // No start detection interrupt (used for I2C)
          0 << USIOIE | // Overflow interrupt disable
          0 << USIWM0 | // No fancy SPI or I2C mode
          1 << USICS0 | // Use Timer 0 as clock
          0 << USICLK;  // No clock strobe
  USISR = 0; // Clear all interrupts and counter
  // Setup Serial output for debug
  serial.begin(115200);
  serial.println("");
  serial.println("Funky LEDs");
}

// USI interrupt
//ISR (USI_OVF_vect) {
//  USISR = 1 << USIOIF | // Clear interrupt flag
//          8;            // Set counter to 8 (will overflow at 16, after 8 samples)
//  // load the current value 
//  PORTB = 16;
//  PORTB = 0;
  
//  tmp = !tmp;
//  uint8_t data = USIDR;
//}

void loop() {
  serial.println(USISR, BIN);
  // Poll the USI counter until it reaches 7, then reset it to 0
  noInterrupts();
  while(true) {
    if (USISR & (1 << USIOIF)) {
      PORTB = 20;
      PORTB = 4;
      USISR = 1 << USIOIF | // Clear interrupt flag
              8;            // Set counter to 8 (will overflow at 16, after 8 samples)
      uint8_t data = USIDR;
      if ((data != 0x00) && (data != 0xFF)) {
        interrupts();
        serial.print("Received ");
        serial.println(data, BIN);
        noInterrupts();
      }
      PORTB = 20;
      PORTB = 4;
    }
  }
  
//  serial.print("Counter val: ");
//  serial.print(TCNT0);
//  serial.print(" / ");
//  serial.print(OCR0A);
//  serial.print(" TIMSK: ");
//  serial.println(TIMSK);
//  if (serial.available()) {
//    uint8_t c = serial.read();
//    serial.print("Received: ");
//    serial.print(c);
//    serial.print(" 0x");
//    serial.print(c, HEX);
//    serial.print(" 0b");
//    serial.print(c, BIN);
//    serial.println("");
    
//    for (int i = 0; i < 8; i++) {
//      leds[i] = (c & (0x01 << i)) ? CRGB::Red : CRGB::Black;
//    }
//  }
  
//  int32_t t = millis() / 12;
//  for (int32_t i = 0; i < NUM_LEDS_TOTAL; i++) {
//    leds[i] = ColorFromPalette(palette, (i * 255 / NUM_LEDS_TOTAL + t) % 255, 255, LINEARBLEND);
//    //leds[i] = CRGB::Red;
//  }
//  FastLED.show();
}
