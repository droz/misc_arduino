#include <FastLED.h>

#define PIN_LEDS 2

#define NUM_LEDS 100
#define X_MAX 10
#define Y_MAX 10

CRGB leds[NUM_LEDS];
static const CRGBPalette16 palette = RainbowColors_p;
enum Dir {UP = 0, LEFT = 1, DOWN = 2, RIGHT = 3};

uint16_t tractor[10] = {
  0b0000111110,
  0b0000100010,
  0b0000100010,
  0b0111111110,
  0b1111111110,
  0b1111111111,
  0b0010000100,
  0b0101001010,
  0b0010000100,
  0b0000000000,
};

void setup() {
  // Setup the LED strip
  FastLED.addLeds<WS2812B, PIN_LEDS, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  // Initialize the random seed
  randomSeed(analogRead(0));
}

// Blank all the pixels on the screen
void Blank() {
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
}

// Fade all the pixels by the same fractional amount
void Fade() {
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i].fadeToBlackBy(2);
  }
}

// Address an LED based on its X/Y coordinates
CRGB& Led(int x, int y) {
  if ((x % 2) == 1) {
    y = 9 - y;
  }
  return leds[y + (x * Y_MAX)];
}

// Move an X/Y point around, and bounce it on a wall when necessary
void MoveAndBounce(int32_t& x, int32_t& y, int32_t& dx, int32_t& dy) {
  x += dx;
  if (x < 0) {
    x = 0;
    dx = -dx;
  }
  if (x >= 256 * X_MAX) {
    x = 256 * X_MAX - 1;
    dx = -dx;
  }
  y += dy;
  if (y < 0) {
    y = 0;
    dy = -dy;
  }
  if (y >= 256 * Y_MAX) {
    y = 256 * Y_MAX - 1;
    dy = -dy;
  }
}
  

void Blobs() {
  static const int NUM_BLOBS = 5;
  int32_t x_b[NUM_BLOBS], y_b[NUM_BLOBS], dx_b[NUM_BLOBS], dy_b[NUM_BLOBS];
  CRGB c_b[NUM_BLOBS];
  // Pick starting points for the blobs, x and y coordinates have the binary point at 8
  for (int i = 0; i < NUM_BLOBS; i++) {
    x_b[i] = random(X_MAX * 256);
    y_b[i] = random(Y_MAX * 256);
    // Make sure we don't pick (0,0) (not moving)
    do {
      dx_b[i] = random(512) - 256;
      dy_b[i] = random(512) - 256;
    } while (dx_b[i] == 0 && dy_b[i] == 0);
    c_b[i] = ColorFromPalette(palette, random(255), 255, LINEARBLEND);
  }
  // Move the blobs around
  for (int i = 0; i < 300; i++) {
    // Now we can start rendering, pixel by pixel.
    for (int32_t x = 0; x < X_MAX; x++) {
      for (int32_t y = 0; y < Y_MAX; y++) {
        CRGB c = CRGB::Black;
        // Go over each blob
        for (int b = 0; b < NUM_BLOBS; b++) {
          // Use the distance to the blob center to fade the color
          int32_t dx = x * 256 - x_b[b];
          int32_t dy = y * 256 - y_b[b];
          if ((dx == 0) && (dy == 0)) {
            c += c_b[b];
          } else {
            int32_t scale = 30000000 / (dx * dx + dy * dy);
            if (scale > 255) {
              scale = 255;
            }
            c += c_b[b] % scale;
          }
        }
        Led(x, y) = c;
      }
    }
    FastLED.show();
    for (int b = 0; b < NUM_BLOBS; b++) {
      MoveAndBounce(x_b[b], y_b[b], dx_b[b], dy_b[b]);
    }
    delay(10);
  }
}

void Snakes() {
  Blank();
  for (int snake = 0; snake < 100; snake++) {
    CRGB color = ColorFromPalette(palette, random(255), 255, LINEARBLEND);
    // Pick random initial direction.
    Dir dir = (Dir) random(4);
    // Pick starting point accordingly.
    int x,y;
    switch(dir) {
      case UP:
        x = random(X_MAX);        
        y = 0;
        break;
      case LEFT:
        x = 9;
        y = random(Y_MAX);
        break;
      case DOWN:
        x = random(X_MAX);        
        y = 9;
        break;
      case RIGHT:
        x = 0;
        y = random(Y_MAX);
        break;
    }
    while(true) {
      Fade();
      Led(x, y) = color;
      FastLED.show();
      delay(10);
      // move the position
      switch(dir) {
        case UP:
          y++;
          break;
        case LEFT:
          x--;
          break;
        case DOWN:        
          y--;
          break;
        case RIGHT:
          x++;
          break;
      }
      if (x < 0 || x >= X_MAX || y < 0 || y >= Y_MAX) {
        break;
      }
      // Decide if we change direction
      if (random(100) < 20) {
        if (random(100) < 50) {
          dir = (Dir) ((dir + 1) % 4);
        } else {
          dir = (Dir) ((dir - 1) % 4);  
        }
      }
    }
  }  
}

void Tractor() {
  Blank();
  for (int r = 0; r < 10; r++) {
    for (int c = 0; c < 10; c++) {
      if ((tractor[r] >> c) & 1) {
        Led(r, c) = CRGB::Green;
      }
    }
  }
  FastLED.show();
  delay(10000);
  
  
}


void loop() {
  Tractor();
  Blobs();
  Snakes();
}
