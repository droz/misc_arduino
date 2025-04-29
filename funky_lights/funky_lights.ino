// NOTE:
//  The ATTINY85 does not use a crystal and per datasheet, its internal oscillator's
//  frequency can vary +/-10%. In practice the bound is much closer, but we should implement
//  a routine which regularly tunes the clock frequency by looking at the position of the serial
//  stream bit transitions.



#include <FastLED.h>
#include <TXOnlySerial.h>
#include <CRC.h>

#define VERBOSE 1

#define LEDS_PIN 1
#define RX_PIN 0
#define TX_DEBUG_PIN 2
#define TIMER_DEBUG_PIN 4

#define NUM_LEDS 30

#define MAGIC_BYTE 0x55
#define CRC_POLYNOMIAL 0x07

#define BROADCAST_UID 0

//#define INITIAL_SERIAL_PRESCALER 208 // 9615 Baud (close to 9600 Baud)
#define INITIAL_SERIAL_PRESCALER 5 // 400 kBaud

// Internal array of LED data
CRGB leds[NUM_LEDS];

// Software serial port, used for debug only
TXOnlySerial debug_serial(TX_DEBUG_PIN);

// The UID for this board. The bar will only answer to messages sent to this specific UID,
//  or to the broadcast UID
// Fixed for now. Should be written in Flash for each board.
uint8_t uid = 42;

// The index of the byte currently being received 
uint8_t byte_index = 0;
// The current state of the deserializer
enum {IDLE, MAGIC, UID, CMD, DATA_LEDS, PRESCALER, CRC} state = IDLE;

//
// Messages formats
//
typedef enum {CMD_LEDS = 1, CMD_SERIAL_BAUDRATE = 2} Cmd;
uint8_t cmd = 0;

// Message: Set serial baudrate
// The baudrate is given as a clock prescaler value. See definition of InitSerial() for details.
typedef struct __attribute__((packed)) SpeedMsg {
  uint8_t magic;
  uint8_t uid;
  Cmd cmd = CMD_SERIAL_BAUDRATE;
  uint8_t prescaler;
  uint8_t crc;
};
uint8_t prescaler;

// Message: Set Leds
typedef uint16_t LedColor;
typedef struct __attribute__((packed)) LedMsg {
  uint8_t magic;
  uint8_t uid;
  Cmd cmd = CMD_LEDS;
  LedColor leds[NUM_LEDS];
  uint8_t crc;
};
LedColor led_colors[NUM_LEDS];
uint8_t* led_colors_bytes = reinterpret_cast<uint8_t *>(led_colors);

// Helper function to print a value in binary WITH leading zeros.
void PrintBinary(uint8_t data, bool line_feed) {
  for (int i = 0; i < 8; i++) {
    debug_serial.print(data >> 7, BIN);
    data <<= 1;
  }
  if (line_feed) {
    debug_serial.println("");
  }
}

// Toggle pin 3 (I/O B4) very quickly. This can be used to get a timing reference on an oscilloscope
// This assumes that the state of the other IOs is know (B0 = 0, B1 = 0, B2 = 1, B3 = 0)
inline void ToggleB4 () {
  PORTB = 0x14;
  PORTB = 0x04;
}

// Configure TIMER0 and the Universal Serial Interface (USI) for a specific baudrate
// The actual baudrate will be given by the formula:
//   baudrate = 16Mhz / (8 * prescaler)
// Speeds up to 400Kb (prescaler = 5) were tested
void InitSerial(uint8_t prescaler) {
  // Timer 0 configuration
  GTCCR = 1 << TSM; // Stop the timer while we are configuring
  OCR0A = prescaler - 1;
  OCR0B = 1;
  TCCR0A = (3 << WGM00);  // Fast PWM mode
  TCCR0B = (1 << WGM02) |  // Fast PWM mode
           (1 << CS00);   // Use clock input directly (no prescaler)
  GTCCR = 0 << TSM; // Start the timer

  // USI configuration register
  USICR = 0 << USISIE | // No start detection interrupt (used for I2C)
          0 << USIOIE | // Overflow interrupt disable
          0 << USIWM0 | // No fancy SPI or I2C mode
          1 << USICS0 | // Use Timer 0 as clock
          0 << USICLK;  // No clock strobe
  USISR = 0; // Clear all interrupts and counter

  interrupts();
  debug_serial.print("Current serial input baudrate: ");
  debug_serial.print(PrescalerToBauds(prescaler));
  debug_serial.println(" Baud");
  noInterrupts();
}
uint32_t PrescalerToBauds(uint8_t prescaler) {
  return 16000000 / (8 * static_cast<uint32_t>(prescaler));
}

// Spin loop until we receive one byte of data from the serial port
inline uint8_t GetSerialByte() {
  uint8_t sample_index = 0;
  uint8_t serial_data = 0;
  uint8_t num_samples = 0;
  uint8_t sampling_mask = 0;

  while(true) {
    // Poll the USI counter interrupt bit
    if (USISR & (1 << USIOIF)) {
      USISR = 1 << USIOIF | // Clear interrupt flag
              8;            // Set counter to 8 (will overflow at 16, after 8 samples)
      uint8_t sample = USIDR;
      if (sample_index == 0) {
        if (sample != 0xFF) {
          sample_index++;
          // Build a one-hot mask where the 1 bit corresponds to the bit 3 samples later than the
          //  first 0 in the input byte. Optimized for speed, use the AVR's SBRC instruction.
          if (!(sample & 0x80)) {
            num_samples = 9;
            sampling_mask = 0b00010000;
          } else if (!(sample & 0x40)) {
            num_samples = 9;
            sampling_mask = 0b00001000;
          } else if (!(sample & 0x20)) {
            num_samples = 9;
            sampling_mask = 0b00000100;
          } else if (!(sample & 0x10)) {
            num_samples = 9;
            sampling_mask = 0b00000010;
          } else if (!(sample & 0x08)) {
            num_samples = 9;
            sampling_mask = 0b00000001;
          } else if (!(sample & 0x04)) {
            num_samples = 10;
            sampling_mask = 0b10000000;
          } else if (!(sample & 0x02)) {
            num_samples = 10;
            sampling_mask = 0b01000000;
          } else {
            sampling_mask = 0b00100000;
            num_samples = 10;
          }  
          serial_data = 0;
        }
      } else {
        sample_index++;
        if (sample_index <= num_samples) {
          serial_data >>= 1;
          if (sample & sampling_mask) {
            serial_data |= 0x80;
          }
        } else {
          return serial_data;
        }
      }
    }
  }
}

void setup() {
  // IO directions
  pinMode(RX_PIN, INPUT);
  pinMode(TIMER_DEBUG_PIN, OUTPUT);
  
  // Setup the LED strip
  FastLED.addLeds<WS2812B, LEDS_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  // Setup Serial output for debug
  debug_serial.begin(115200);
  debug_serial.println("");
  debug_serial.println("Funky LEDs");
  debug_serial.print("Number of LEDs: ");
  debug_serial.println(NUM_LEDS);
  debug_serial.print("UID: ");
  debug_serial.println(uid);  

  // Setup the input serial port
  InitSerial(INITIAL_SERIAL_PRESCALER);

  noInterrupts();
}

void loop() {

  // Wait for messages
  uint8_t c = GetSerialByte();
  switch (state) {
    case IDLE:
      if (c == MAGIC_BYTE) {
        state = MAGIC;
      } else {
        if (VERBOSE) {
          interrupts();
          debug_serial.print("Bad Magic: 0x");
          debug_serial.println(c, HEX);
          noInterrupts();
        }
      }
      break;
    case MAGIC:
      if (c == uid || c == BROADCAST_UID) {
        state = CMD;
        byte_index = 0;
      } else {
        if (VERBOSE) {
          interrupts();
          debug_serial.print("Bad UID: ");
          debug_serial.println(c);
          noInterrupts();
        }
        state = IDLE;
      }
      break;
    case CMD:
      cmd = c;
      switch(c) {
        case CMD_LEDS:
          state = DATA_LEDS;
          break;
        case CMD_SERIAL_BAUDRATE:
          state = PRESCALER;
          break;
        default:
          if (VERBOSE) {
            interrupts();
            debug_serial.print("Bad command: 0x");
            debug_serial.println(c, HEX);
            noInterrupts();
          }
          state = IDLE;
          break;
        }
      break;
    case DATA_LEDS:
      led_colors_bytes[byte_index++] = c;
      if (byte_index == NUM_LEDS * 2) {
        state = CRC;
      }
      break;
    case PRESCALER:
      prescaler = c;      
      break;
    case CRC:
      state = IDLE;
      // compute what the CRC should be
      uint8_t crc = 0;
      switch (cmd) {
        case CMD_LEDS:
          crc = crc8(led_colors_bytes, sizeof(led_colors), CRC_POLYNOMIAL);
          break;
        case CMD_SERIAL_BAUDRATE:
          crc = crc8(&prescaler, 1, CRC_POLYNOMIAL);
          break;
      }
      if (c == crc) {
        switch (cmd) {
          case CMD_LEDS:
            for (int i = 0; i < NUM_LEDS; i++) {
              uint16_t col = led_colors[i];
              leds[i] = CRGB((col >> 8) & 0xF8, (col >> 3) & 0xFC, (col << 3) & 0xF8);
            }
            break;
        case CMD_SERIAL_BAUDRATE:
        }
      } else {
        if (VERBOSE) {
          interrupts();
          debug_serial.print("Bad CRC: 0x");
          debug_serial.println(c, HEX);
          noInterrupts();
        }
      }
      interrupts();
      FastLED.show();
      noInterrupts();
      break;
  }
}
