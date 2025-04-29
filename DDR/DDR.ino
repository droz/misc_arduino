/* Dance Dance Revolution Pad code */

#include <HX711_ADC.h>
#include "FastLED.h"

#define DEBUG
#define WITH_LEDS

#define LOAD_CALIBRATION -46900
#define LOAD_THRESHOLD_KG 3.0f

#define CLK0       2
#define DOUT0      3
#define CLK1       4
#define DOUT1      5
#define CLK2       6
#define DOUT2      7
#define CLK3       8
#define DOUT3      9
#define DBG_PIN    11
#define LED_PIN    13
#define OUT_PIN    12
#define NUM_LEDS_X 16
#define NUM_LEDS_Y 16
#define NUM_LEDS (NUM_LEDS_X * NUM_LEDS_Y)

HX711_ADC sensor0(DOUT0, CLK0);
HX711_ADC sensor1(DOUT1, CLK1);
HX711_ADC sensor2(DOUT2, CLK2);
HX711_ADC sensor3(DOUT3, CLK3);

// The readings from the load cells
float sensor0_load_kg = 0.0f;
float sensor1_load_kg = 0.0f;
float sensor2_load_kg = 0.0f;
float sensor3_load_kg = 0.0f;

// The value we load in timer1_counter everytime we get interrupted
#define TIMER1_VAL 65523   // 65536 - (16MHz/256 * 200us)

// Palette used for color interpolation
static const CRGBPalette16 palette = RainbowColors_p;

// The array of LEDs
CRGB leds[NUM_LEDS];

// Time management
int32_t last_time = 0;

// Track past touches
bool touching = false;
int32_t last_touch_time = 0;
float last_touch_x = 0.0f;
float last_touch_y = 0.0f;
CRGB last_touch_color = CRGB::Black;

//
// Utility functions
//

// Blank all the pixels on the screen
void Blank() {
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
}

// Fade all the pixels by the same fractional amount
void Fade(int factor) {
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i].fadeToBlackBy(factor);
  }
}

// Address an LED based on its X/Y coordinates
CRGB& Led(int x, int y) {
  if ((x % 2) == 1) {
    y = NUM_LEDS_Y - 1 - y;
  }
  return leds[y + (x * NUM_LEDS_Y)];
}
void SetLed(int x, int y, const CRGB& c) {
  if (x < 0 || x >= NUM_LEDS_X || y < 0 || y >= NUM_LEDS_Y) {
    return;
  }
  Led(x, y) = c;
}
  
// Pick a color related to time
CRGB Color(CRGB color) {
  if (color) {
    return color;
  } else {
    int32_t t = millis();
    return ColorFromPalette(palette, (t / 300) % 256, 255, LINEARBLEND);
  }
}

// Raster a circle using Bresenham's algorith
void Draw8Pixels(int x_center, int y_center, int x, int y, const CRGB& c) {
  SetLed(x_center + x, y_center + y, c);
  SetLed(x_center - x, y_center + y, c);
  SetLed(x_center + x, y_center - y, c);
  SetLed(x_center - x, y_center - y, c);
  SetLed(x_center + y, y_center + x, c);
  SetLed(x_center - y, y_center + x, c);
  SetLed(x_center + y, y_center - x, c);  
  SetLed(x_center - y, y_center - x, c);
}
void Circle(int x_center, int y_center, int r, const CRGB& c) {
  int y = r;
  int d = 3 - 2 * r;
  Draw8Pixels(x_center, y_center, 0, r, c);
  for (int x = 0; y >= x; x++) {
    if (d > 0) {
      y--;
      d = d + 4 * (x - y) + 10;
    } else {
      d = d + 4 * x + 6;
    }
    Draw8Pixels(x_center, y_center, x, y, c);
  }
}

//
// Visual effects
//

template <int NUM_BLOBS>
class Blob {
 public:
  void Init() {
    for (int i = 0; i < NUM_BLOBS; i++) {
      ResetBlob(i);
    }
  }

  // All coordinates and size in units of 256th of a pixel
  void ResetBlob(int n) {
    x_b_[n] = 0;
    y_b_[n] = 0;
    size_b_[n] = 0;
    c_b_[n] = CRGB::Black;    
  }
  
  void SetBlob(int n, int x, int y, uint32_t r, const CRGB& c) {
    x_b_[n] = x;
    y_b_[n] = y;
    if (r == 0) {
      size_b_[n] = 0;
    } else {
      size_b_[n] = 16777216 / (r * r);
    }
    c_b_[n] = c;
    Serial.println(size_b_[n]);
  }

  void Draw(int dt) {
    // Go over all the pixels and decide a color for each of them.
    for (int32_t x = 0; x < NUM_LEDS_X; x++) {
      for (int32_t y = 0; y < NUM_LEDS_Y; y++) {
        CRGB c = Led(x, y);
        // Go over each blob
        for (int b = 0; b < NUM_BLOBS; b++) {
          if (size_b_[b] == 0) {
            continue;
          }
          // Use the distance to the blob center to fade the color
          int32_t dx = x * 16 - x_b_[b];
          int32_t dy = y * 16 - y_b_[b];
          int32_t scale = 255 - (((dx * dx + dy * dy) * size_b_[b]) >> 16);
          if (scale > 255) {
            scale = 255;
          }
          if (scale < 0) {
            scale = 0;
          }
          if (scale > 160) {
            scale = 0;
          } else if (scale > 128) {
            scale = 255 - (160 - scale) * 8;
          }
          
          CRGB c_r = CRGB::Red;
          c_r.nscale8(scale);
          c += c_b_[b] % scale;
          //c += c2;
          
          //CRGB c_p = c_b_[b];
          //c_p.nscale8(scale);
          //c += c_p;
        }
        Led(x, y) = c;
      }
    }

  }
 private:
  // Blob coordinates. Units in 256th of a pixel.
  int32_t x_b_[NUM_BLOBS];
  int32_t y_b_[NUM_BLOBS];
  // Blob velocities.
  //int32_t dx_b[NUM_BLOBS];
  //int32_t dy_b[NUM_BLOBS];
  // Blob colors
  CRGB c_b_[NUM_BLOBS];
  // Blob size. We store the inverse of the size.
  uint32_t size_b_[NUM_BLOBS];
};
// The blob used to display touches
Blob<1> touch_blob;



class Rain {
 public:
  void Init() {
    for (int i = 0; i < NUM_DROPLETS; i++) {
      droplet_x_[i] = 0;
      droplet_y_[i] = (NUM_LEDS_Y + 1) * DROPLET_SPEED;
    }
  }

  void Draw(int dt) {
    for (int i = 0; i < NUM_DROPLETS; i++) {
      if(droplet_y_[i] >= NUM_LEDS_Y * 256) {
        droplet_y_[i] = - random(NUM_LEDS_Y * 256);
        droplet_x_[i] = random(NUM_LEDS_X * 256);
        droplet_c_[i] = ColorFromPalette(palette, random(255), 255, LINEARBLEND);
      } else {
        droplet_y_[i] += DROPLET_SPEED * dt;
      }
      int x = droplet_x_[i] / 256;
      int y = droplet_y_[i] / 256;
      if (x >= 0 && x < NUM_LEDS_X) {
        for (int j = 0; j < 8; j++) {
          if (y - j >= 0 && y - j < NUM_LEDS_Y) {
            uint8_t scale = 1 << j;
            Led(x, y - j) = droplet_c_[i] / scale;
          }
        }
      }
    }    
  }
 private:
  static const int NUM_DROPLETS = 10;
  // Speed in pixel / 256ms
  static const int DROPLET_SPEED = 10;
  // Position in pixels / 256
  int droplet_x_[NUM_DROPLETS];
  int droplet_y_[NUM_DROPLETS];
  CRGB droplet_c_[NUM_DROPLETS];
 
} rain;


const uint16_t arrow[16] = {
  0x0100, // 0000000100000000
  0x0300, // 0000001100000000
  0x0700, // 0000011100000000
  0x0F00, // 0000111100000000
  0x1FFE, // 0001111111111111
  0x3FFE, // 0011111111111111
  0x7FFE, // 0111111111111111
  0xFFFE, // 1111111111111111
  0xFFFE, // 1111111111111111
  0x7FFE, // 0111111111111111
  0x3FFE, // 0011111111111111
  0x1FFE, // 0001111111111111
  0x0F00, // 0000100100000000
  0x0700, // 0000011100000000
  0x0300, // 0000001100000000
  0x0100, // 0000000100000000
};


void setup() {
  // Serial port setup
  Serial.begin(115200);
  Serial.println("Magic Scale ! Running HX711_ADC lib");

  // Debug pin
  pinMode(DBG_PIN, OUTPUT);
  digitalWrite(DBG_PIN, LOW);

  // Output pin to the joystick control board
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  // LEDs setup
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  Blank();
  FastLED.show();

  // Initialize the visual effects
  rain.Init();
  touch_blob.Init();

  // Initialize the load cell amplifiers and do a tare cycle
  sensor0.begin();
  sensor1.begin();
  sensor2.begin();
  sensor3.begin();
  sensor0.setCalFactor(LOAD_CALIBRATION);
  sensor1.setCalFactor(LOAD_CALIBRATION);
  sensor2.setCalFactor(LOAD_CALIBRATION);
  sensor3.setCalFactor(LOAD_CALIBRATION);
  delay(100);
  sensor0.start(2000, true);
  sensor1.start(2000, true);
  sensor2.start(2000, true);
  sensor3.start(2000, true);
  Serial.println("Sensors calibrated");

  // From now on, we need to service the load cell sensors very quickly (<12ms) after they have new data.
  // But we have long tasks (like filling the positions of LEDs) that will take a while, so we setup timer1 to
  // allow us to check every 200us.
  // FastLED will mask the interrupts and disable the timer while it is sending data, but that lasts only for 8ms,
  // which should still give us enough time for the hx711.
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  
  TCNT1 = TIMER1_VAL;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts 

}

// Timer 1 interrupt routine
ISR(TIMER1_OVF_vect)
{
  TCNT1 = TIMER1_VAL;   // preload timer
  // Check all sensors, and read them if necessary
  if (sensor0.update()) {
    sensor0_load_kg = sensor0.getData();
  }
  if (sensor1.update()) {
    sensor1_load_kg = sensor1.getData();
  }
  if (sensor2.update()) {
    sensor2_load_kg = sensor2.getData();
  }
  if (sensor3.update()) {
    sensor3_load_kg = sensor3.getData();
  }

  float sum = sensor0_load_kg + sensor1_load_kg + sensor2_load_kg + sensor3_load_kg;
  if (sum > LOAD_THRESHOLD_KG) {
    digitalWrite(OUT_PIN, HIGH);
  } else {
    digitalWrite(OUT_PIN, LOW);    
  }

  // A tick to see when the interrupt is happening
  digitalWrite(DBG_PIN, HIGH);
  digitalWrite(DBG_PIN, LOW);    
}

//
// Main Loop
//

void loop() {
  int32_t now = millis();
  int32_t dt = now - last_time;
  last_time = now;
  
  // The sensors are being read asynchronously by timer1. To make sure that the values do not change while
  // we are using them, we make a local copy here.
  noInterrupts();
  float l0 = sensor0_load_kg;
  float l1 = sensor1_load_kg;
  float l2 = sensor2_load_kg;
  float l3 = sensor3_load_kg;
  interrupts();

  // Compute the location of the center of the force applied to the pad
  float sum = l0 + l1 + l2 + l3;
  float x = (l1 + l2) / sum * (NUM_LEDS_X - 1);
  float y = (l2 + l3) / sum * (NUM_LEDS_Y - 1);

  // Track touches
  int32_t touch_dt = now - last_touch_time;
  if (sum > LOAD_THRESHOLD_KG) {
    if (!touching) {
      last_touch_time = now;
      last_touch_x = x;
      last_touch_y = y;
      last_touch_color = ColorFromPalette(palette, random(255), 255, LINEARBLEND);
    }
    touching = true;   
  } else {
    touching = false;
  }
  if (touch_dt < 1000) {
    CRGB c = CRGB::Green;
    touch_blob.SetBlob(0, last_touch_x * 16, last_touch_y * 16, touch_dt / 2, last_touch_color);
  } else {
    touch_blob.ResetBlob(0);    
  }

  // Display the background
  Blank();

  rain.Draw(dt);
  touch_blob.Draw(dt);
#ifdef WITH_LEDS
  FastLED.show();
#endif
#ifdef DEBUG
  static float last_t = 0.0;
  float t = static_cast<float>(millis()) / 1000.0;
  if (t - last_t > 1.0) {
    last_t = t;
    Serial.print(t);
    Serial.print(" ");
    Serial.print(l0);
    Serial.print(" ");
    Serial.print(l1);
    Serial.print(" ");
    Serial.print(l2);
    Serial.print(" ");
    Serial.print(l3);
    Serial.println("");
  }

#endif
  
}
