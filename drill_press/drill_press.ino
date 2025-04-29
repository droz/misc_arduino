#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>

#define ENA_PIN 4
#define DIR_PIN 3
#define PUL_PIN 2
#define PUSH0_PIN 9
#define PUSH1_PIN 11
#define PUSH2_PIN A0
#define PUSH3_PIN A2
#define LED0_PIN 10
#define LED1_PIN 12
#define LED2_PIN A1
#define LED3_PIN A3

// Stepper parameters
const uint32_t kMaxSpeed = 2000; // ticks/s
const uint32_t kMaxAccel = 10000; // ticks/s/s

// Time before we go to sleep, in ms
const uint32_t kSleepTime = 600000;

// The last time we updated the LCD
uint32_t last_lcd_update = 0;

// The LCD display object
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

// The stepper motor controller object
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN, 0, 0);

// Number of ticks per revolutions
const int kTicksPerRev = 400;
// The pitch of the lead screw, in mm
const float kScrewPitchMm = 8.0;
// One inch in mm
const float kInchInMm = 25.4;

// The conversion from steps to mm and inches
const float kStepInMm = kScrewPitchMm / kTicksPerRev;
const float kStepInInch = kStepInMm / kInchInMm;

// Increment sizes
const float kIncrementMm = 1.0;
const float kIncrementInch = 1.0 / 32;

// These track when a button was last pressed.
uint32_t push0_time = 0;
uint32_t push1_time = 0;
uint32_t push2_time = 0;
uint32_t push3_time = 0;
// The last know state of each button
bool pushed0 = false;
bool pushed1 = false;
bool pushed2 = false;
bool pushed3 = false;

// Tells us what kind of move we are currently executing
enum State {STOPPED = 0, FAST = 1, INCREMENTAL = 2, SLEEP = 3} state = SLEEP;

// What unit are we currently using
enum Unit {MM = 0, INCH_DEC = 1, INCH_FRAC = 2} units = MM; 

// The value we load in timer1_counter everytime we get interrupted
#define TIMER1_VAL 65529   // 65536 - (16MHz/256 * 100us)

void setup() {
  Serial.begin(115200);

  // Stepper setup
  pinMode(ENA_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);
  stepper.setMaxSpeed(kMaxSpeed);
  stepper.setAcceleration(kMaxAccel);
  stepper.moveTo(0);

  // Buttons setup
  pinMode(PUSH0_PIN, INPUT_PULLUP); 
  pinMode(PUSH1_PIN, INPUT_PULLUP); 
  pinMode(PUSH2_PIN, INPUT_PULLUP); 
  pinMode(PUSH3_PIN, INPUT_PULLUP); 
  pinMode(LED0_PIN, OUTPUT); 
  pinMode(LED1_PIN, OUTPUT); 
  pinMode(LED2_PIN, OUTPUT); 
  pinMode(LED3_PIN, OUTPUT); 
  digitalWrite(LED0_PIN, HIGH);
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
  digitalWrite(LED3_PIN, HIGH);

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.display();
  // Add up and down arrows
  const uint8_t up[] = {
    0b00000,
    0b00100,
    0b01110,
    0b11111,
    0b10101,
    0b00100,
    0b00100,
    0b00100,
  };
  const uint8_t down[] = {
    0b00000,
    0b00100,
    0b00100,
    0b00100,
    0b10101,
    0b11111,
    0b01110,
    0b00100,
  };
  lcd.createChar(1, up);
  lcd.createChar(2, down);
  // Add hare icon
  const uint8_t hare0[] = {
    0b00011,
    0b00110,
    0b01100,
    0b11111,
    0b11111,
    0b00111,
    0b01100,
    0b10000,
  };
  const uint8_t hare1[] = {
    0b00000,
    0b00000,
    0b00010,
    0b11110,
    0b11110,
    0b11100,
    0b00111,
    0b00000,
  };
  lcd.createChar(3, hare0);
  lcd.createChar(4, hare1);
  // Add home icon
  const uint8_t home[] = {
    0b00000,
    0b00100,
    0b01110,
    0b11111,
    0b11111,
    0b10001,
    0b10001,
    0b10001,
  };
  lcd.createChar(5, home);
  // Add units icon
  const uint8_t units[] = {
    0b00000,
    0b01110,
    0b00100,
    0b00110,
    0b01100,
    0b00110,
    0b00100,
    0b01110,
  };
  lcd.createChar(6, units);

  // We need to call the steppers run() function at a minimum of 3kHz.
  // The LCD takes longer than that to refresh though so we are going to setup
  // a timer interrupt running at 10kHz to take care of it.
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = TIMER1_VAL;       // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

  GoToSleep();
}

// Timer 1 interrupt routine
ISR(TIMER1_OVF_vect)
{
  TCNT1 = TIMER1_VAL;   // preload timer
  stepper.run();
}

void UpdatePush(uint8_t pin, uint32_t* t, bool* pushed, bool* pressed, bool* released) {
  bool pushed_state = !digitalRead(pin);
  *pressed = false;
  *released = false;
  if (pushed_state) {
    if (!*pushed) {
      *t = millis();
      *pressed = true;
    }
  } else {
    if (*pushed) {
      *released = true;
    }
  }
  *pushed = pushed_state;
}

// This assumes that the string is already allocated long enough !
void PadString(uint8_t length, char* str) {
  int i;
  for (i = strlen(str); i < length; i++) {
    str[i] = ' ';
  }
  str[i] = 0;
}

void ToFraction(float remainder, uint8_t base, char* frac) {
  uint8_t denom = base;
  uint8_t num = round(remainder * base);
  while (num % 2 == 0 && num != 0) {
    num /= 2;
    denom /= 2;
  }
  if (num == 0) {
    sprintf(frac, "");
    return;
  }
  sprintf(frac, "%d/%d", num, denom);
}

// These functions wrap the stepper API and make sure that we are always turning off
//  interrupts before entering stepper code.
void StepperMove(long relative) {
  noInterrupts();
  stepper.move(relative);
  interrupts();
}
void StepperMoveTo(long absolute) {
  noInterrupts();
  stepper.moveTo(absolute);
  interrupts();
}
long StepperCurrentPosition() {
  noInterrupts();
  long val = stepper.currentPosition();
  interrupts();
  return val;
}
void StepperStop() {
  noInterrupts();
  stepper.stop();
  interrupts();
}
long StepperDistanceToGo() {
  noInterrupts();
  long val = stepper.distanceToGo();
  interrupts();
  return val;
}
void StepperSetCurrentPosition(long position) {
  noInterrupts();
  stepper.setCurrentPosition(position);
  interrupts();
}

void GoToSleep() {
  state = SLEEP;
  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(LED0_PIN, LOW);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("  Drill-o-tron 9000 ");
  lcd.setCursor(0, 3);
  lcd.print("\02     \02      \02     \02");
  lcd.noBacklight();
}

void loop() {
  uint32_t now = millis();
  bool push0_pressed, push1_pressed, push2_pressed, push3_pressed;
  bool push0_released, push1_released, push2_released, push3_released;
  UpdatePush(PUSH0_PIN, &push0_time, &pushed0, &push0_pressed, &push0_released);
  UpdatePush(PUSH1_PIN, &push1_time, &pushed1, &push1_pressed, &push1_released);
  UpdatePush(PUSH2_PIN, &push2_time, &pushed2, &push2_pressed, &push2_released);
  UpdatePush(PUSH3_PIN, &push3_time, &pushed3, &push3_pressed, &push3_released);

  int32_t increment_ticks = (units == MM) ? kIncrementMm / kStepInMm : kIncrementInch / kStepInInch;

  int32_t pos = StepperCurrentPosition();
  switch (state) {
    case SLEEP:
      if (push0_pressed || push1_pressed || push2_pressed || push3_pressed) {
        state = STOPPED;
        digitalWrite(ENA_PIN, LOW);
        digitalWrite(LED0_PIN, HIGH);
        digitalWrite(LED1_PIN, HIGH);
        digitalWrite(LED2_PIN, HIGH);
        digitalWrite(LED3_PIN, HIGH);
        lcd.clear();
        lcd.backlight();
      }
      break;
    case STOPPED:
      // if we have not touched a button in a while, we can go to sleep
      if ((now - push0_time) > kSleepTime &&
          (now - push1_time) > kSleepTime &&
          (now - push2_time) > kSleepTime &&
          (now - push3_time) > kSleepTime) {
            GoToSleep();
            break;
          }
      if (pushed3) {
        if (push1_pressed) {
          StepperMove(1000000);
          state = FAST;
        }
        if (push2_pressed) {
          StepperMove(-1000000);
          state = FAST;
        }
      } else {
        int32_t next_pos = pos;
        if (push1_pressed || push2_pressed) {
          if (units == MM) {
            int32_t current_pos_inc = round(static_cast<float>(pos) * kStepInMm / kIncrementMm);
            if (push1_pressed) current_pos_inc++;
            if (push2_pressed) current_pos_inc--;
            next_pos = current_pos_inc * kIncrementMm / kStepInMm;
          } else {
            int32_t current_pos_inc = round(static_cast<float>(pos) * kStepInInch / kIncrementInch);
            if (push1_pressed) current_pos_inc++;
            if (push2_pressed) current_pos_inc--;
            next_pos = current_pos_inc * kIncrementInch / kStepInInch;
          }
          StepperMoveTo(next_pos);
          state = INCREMENTAL;
        }
      }
      break;
    case FAST:
      if (push1_released || push2_released || !pushed3) {
        StepperStop();
        state = STOPPED;
      }
      break;
    case INCREMENTAL:
      if (StepperDistanceToGo() == 0) {
        state = STOPPED;
      }
      break;
  }

  if (push0_released && now - push0_time < 1000) {
    if (units == MM) {
      units = INCH_DEC;
    } else if (units == INCH_DEC) {
      units = INCH_FRAC;
    } else {
      units = MM;
    }
  }
  if (pushed0 && now - push0_time > 1000) {
    StepperSetCurrentPosition(0);
  }
  
  // Display.
  if (now - last_lcd_update > 100 && state != SLEEP) {
    char str[21];
    bool negative = false;
    last_lcd_update = now;
    lcd.backlight();
    switch (units) {
      case MM: {
        float pos_mm = kStepInMm * pos;
        if (pos_mm < 0) {
          negative = true;
          pos_mm = - pos_mm;
        }
        int32_t pos_um = static_cast<int32_t>(pos_mm * 1000.0);
        sprintf(str, "      %s%ld.%03ld mm", negative ? "-" : "", pos_um / 1000, abs(pos_um % 1000));
        PadString(20, str);
        lcd.setCursor(0, 1);
        lcd.print(str);
        break;
      }
      case INCH_DEC: {
        float pos_in = kStepInInch * pos;
        if (pos_in < 0) {
          negative = true;
          pos_in = - pos_in;
        }
        int32_t pos_thou = static_cast<int32_t>(pos_in * 1000.0);
        int16_t pos_decimals = abs(pos_thou % 1000);
        sprintf(str, "      %s%ld.%03d\"", negative ? "-" : "", pos_thou / 1000, pos_decimals);
        PadString(20, str);
        lcd.setCursor(0, 1);
        lcd.print(str);
        break;
      }
      case INCH_FRAC: {
        float pos_in = kStepInInch * pos;
        if (pos_in < 0) {
          negative = true;
          pos_in = - pos_in;
        }
        int32_t pos_int;
        float pos_decimals;
        if (pos_in >= 0.0) {
          pos_int = floor(pos_in);
          pos_decimals = pos_in - pos_int;
        } else {
          pos_int = ceil(pos_in);
          pos_decimals = pos_int - pos_in; 
        }
        char frac[6];
        ToFraction(pos_decimals, 64, frac);
        sprintf(str, "      %s%ld\" %s", negative ? "-" : "", pos_int, frac);
        PadString(20, str);
        lcd.setCursor(0, 1);
        lcd.print(str);
        break;
      }
    }
    lcd.setCursor(0, 3);
    lcd.print("\06/\05   \01      \02    \03\04");
  }  
}
