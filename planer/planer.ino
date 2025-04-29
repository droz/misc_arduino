#include <FastAccelStepper.h>
#include <LiquidCrystal_I2C.h>

#define ENA_PIN 8
#define DIR_PIN 7
#define PUL_PIN 9
#define PUSH0_PIN 4
#define PUSH1_PIN 2
#define PUSH2_PIN A1
#define PUSH3_PIN A3
#define LED0_PIN 5
#define LED1_PIN 3
#define LED2_PIN A0
#define LED3_PIN A2
#define GPIO2_PIN 10
#define GPIO3_PIN 6
#define LCD_SDA A4
#define LCD_SCL A5 

// This is the height of the top limit switch in mm
const float kTopHeight = 154.75;   // mm

// This is the rest position we should go to after homing
const float kRestHeight = 20.00;   // mm

// These are the up and down boundaries
const float kMaxHeight = 150.00; // mm
const float kMinHeight = 3.00;   // mm
const float kFastMargin = 5.00;  // mm

// Stepper parameters
const uint32_t kMaxSpeed = 200;    // us/step
const uint32_t kHomeSpeed = 2000;  // us/step
const uint32_t kMaxAccel = 20000;  // ticks/s/s

// Time before we go to sleep, in ms
const uint32_t kSleepTime = 600000;
// Time before we release the enable of the stepper, in ms
const uint32_t kStepperReleaseTime = 320000;

// The last time we updated the LCD
uint32_t last_lcd_update = 0;

// The LCD display object
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

// The stepper motor controller object
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Number of ticks per revolutions
const int kTicksPerRev = 400;
// The pitch of the lead screw, in mm
const float kScrewPitchMm = 0.52662259648;
// One inch in mm
const float kInchInMm = 25.4;

// The conversion from steps to mm and inches
const float kStepInMm = kScrewPitchMm / kTicksPerRev;
const float kStepInInch = kStepInMm / kInchInMm;

// Increment sizes
const float kIncrementMm = 0.25;
const float kIncrementInch = 1.0 / 64;

// The tick we used as a zero mark
int32_t zero_pos = 0;

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
enum State {STOPPED = 0, FAST = 1, INCREMENTAL = 2, SLEEP = 3} state = STOPPED;

// What unit are we currently using
enum Unit {MM = 0, INCH_DEC = 1, INCH_FRAC = 2} units = MM; 

// Reset function (jump to address 0)
void(* resetFunc) (void) = 0;

void setup() {
  Serial.begin(115200);

  // Steppe2r setup
  engine.init();
  stepper = engine.stepperConnectToPin(PUL_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENA_PIN);
    stepper->setAutoEnable(true);
    stepper->setSpeedInUs(kMaxSpeed);
    stepper->setAcceleration(kMaxAccel);
    stepper->setDelayToEnable(0);
    stepper->setDelayToDisable(kStepperReleaseTime);
    stepper->disableOutputs();
    Serial.println("INIT DONE");
  } else {
    Serial.println("INIT FAILED");
  } 

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

  // Limit Switch
  pinMode(GPIO2_PIN, INPUT_PULLUP); 
  pinMode(GPIO3_PIN, INPUT_PULLUP); 

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

  // Wait for OK before homing
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Plan-o-matic 9000 ");
  lcd.setCursor(0, 1);
  lcd.print("       HOME ?");
  lcd.setCursor(0, 3);
  lcd.print("UP              DOWN");
  bool stay_up = true;
  while(true) {
    if (!digitalRead(PUSH3_PIN)) {
      stay_up = false;
      break;
    }
    if (!digitalRead(PUSH0_PIN)) {
      stay_up = true;
      break;
    }
    delay(1);
  };
  lcd.clear();
  bool home_success = Home(stay_up);
  if (!home_success) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("    FAILED HOME !");
    while(true) {
      delay (1000);
    }
  }
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

void MmToString(int32_t pos, char* str) {
  bool negative = false;
  float pos_mm = kStepInMm * pos;
  if (pos_mm < 0) {
    negative = true;
    pos_mm = - pos_mm;
  }
  int32_t pos_um = static_cast<int32_t>(pos_mm * 1000.0);
  // Round to the closest 5um
  pos_um = (pos_um + 2) / 5 * 5;
  sprintf(str, "      %s%ld.%03ld mm", negative ? "-" : "", pos_um / 1000, abs(pos_um % 1000));
  PadString(20, str);
}

void InchDecToString(int32_t pos, char* str) {
  bool negative = false;
  float pos_in = kStepInInch * pos;
  if (pos_in < 0) {
    negative = true;
    pos_in = - pos_in;
  }
  int32_t pos_thou = static_cast<int32_t>(pos_in * 1000.0);
  int16_t pos_decimals = abs(pos_thou % 1000);
  sprintf(str, "      %s%ld.%03d\"", negative ? "-" : "", pos_thou / 1000, pos_decimals);
  PadString(20, str);
}

void InchFracToString(int32_t pos, char* str) {
  bool negative = false;
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
}

// This function makes sure that we give enough time between enabling the stepper and moving
void StepperTurnOnAndMove(long relative) {
  if (digitalRead(ENA_PIN)) {
    stepper->enableOutputs();
    delay(1000);
  }
  stepper->move(relative);
}
void StepperTurnOnAndMoveTo(long absolute) {
  if (digitalRead(ENA_PIN)) {
    stepper->enableOutputs();
    delay(1000);
  }
  stepper->moveTo(absolute);
}

// Wait for the stepper to be done moving
void WaitStopped() {
  while(stepper->isRunning()) {
    delay(1);
  }
}

// Tells us if the limit switch is triggered
bool AtLimit() {
  return digitalRead(GPIO3_PIN);
}

bool Home(bool stay_up) {
  const uint32_t timeout_fast_ms = 25000;
  const uint32_t timeout_slow_ms = 10000;
  uint32_t start_time;
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("  Coarse search...");      
  stepper->setSpeedInUs(kMaxSpeed);
  if (!AtLimit()) {
    stepper->move(-1000000);
  }
  start_time = millis();
  while(!AtLimit()) {
    uint32_t now = millis();
    if (now - start_time > timeout_fast_ms) {
      stepper->stopMove();
      return false;
    }
    delay(1);
  }
  stepper->stopMove();
  WaitStopped();
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("   Backing up...");      
  stepper->move(3000);
  WaitStopped();
  if (AtLimit()) {
    stepper->stopMove();
    return false;
  }
  stepper->setSpeedInUs(kHomeSpeed);
  stepper->move(-5000);
  start_time = millis();
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("   Fine search...");      
  while(true) {
    uint32_t now = millis();
    if (now - start_time > timeout_slow_ms) {
      stepper->stopMove();
      return false;
    }
    delay(1);
    if (AtLimit()) {
      stepper->setCurrentPosition(- kTopHeight / kStepInMm);
      break;
    }
  }
  stepper->stopMove();
  WaitStopped();
  stepper->setSpeedInUs(kMaxSpeed);
  if (!stay_up) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("   Moving down...");
    stepper->moveTo(- kRestHeight / kStepInMm);
    WaitStopped();
  }
  
  lcd.clear();
  return true;
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

  int32_t pos = - stepper->getCurrentPosition();
  int32_t current_pos_inc = 0;
  if (units == MM) {
    current_pos_inc = round(static_cast<float>(pos - zero_pos) * kStepInMm / kIncrementMm);
  } else {
    current_pos_inc = round(static_cast<float>(pos - zero_pos) * kStepInInch / kIncrementInch);
  }
  
  switch (state) {
    case SLEEP:
      if (push0_pressed || push1_pressed || push2_pressed || push3_pressed) {
        state = STOPPED;
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
            state = SLEEP;
            digitalWrite(LED0_PIN, LOW);
            digitalWrite(LED1_PIN, LOW);
            digitalWrite(LED2_PIN, LOW);
            digitalWrite(LED3_PIN, LOW);
            lcd.clear();
            lcd.setCursor(0, 1);
            lcd.print(" Plan-o-matic 9000 ");
            lcd.setCursor(0, 3);
            lcd.print("\02     \02      \02     \02");
            lcd.noBacklight();
            break;
          }
      if (pushed3) {
        if (push1_pressed) {
          StepperTurnOnAndMove(-1000000);
          state = FAST;
        }
        if (push2_pressed) {
          StepperTurnOnAndMove(1000000);
          state = FAST;
        }
      } else {
        int32_t next_pos = pos;
        if (push1_pressed || push2_pressed) {
          if (push1_pressed) current_pos_inc++;
          if (push2_pressed) current_pos_inc--;
          if (units == MM) {
            next_pos = current_pos_inc * kIncrementMm / kStepInMm + zero_pos;
          } else {
            next_pos = current_pos_inc * kIncrementInch / kStepInInch + zero_pos;
          }
          StepperTurnOnAndMoveTo(-next_pos);
          state = INCREMENTAL;
        }
      }
      break;
    case FAST:
      if (push1_released || push2_released || !pushed3) {
        stepper->stopMove();
        state = STOPPED;
      }
      break;
    case INCREMENTAL:
      if (!stepper->isRunning() ) {
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
    zero_pos = -stepper->getCurrentPosition();
  }
  if (pushed0 && now - push0_time > 5000) {
    resetFunc();
  }

  // Implement upper and lower bounds. When we are going fast, we should be more restrictive on the bounds
  int32_t max_pos = kMaxHeight / kStepInMm;
  int32_t min_pos = kMinHeight / kStepInMm;
  if (state == FAST) {
    max_pos -= kFastMargin / kStepInMm;
    min_pos += kFastMargin / kStepInMm;
  }
  if (stepper->targetPos() < stepper->getCurrentPosition() && pos > max_pos) {
    stepper->stopMove();
    WaitStopped();
  }
  if (stepper->targetPos() > stepper->getCurrentPosition() && pos < min_pos) {
    stepper->stopMove();
    WaitStopped();
  }

  
  // Display.
  if (now - last_lcd_update > 100 && state != SLEEP) {
    char str[21];
    last_lcd_update = now;
    lcd.backlight();
    switch (units) {
      case MM: {
        MmToString(pos, str);
        lcd.setCursor(0, 1);
        lcd.print(str);
        MmToString(pos - zero_pos, str);
        lcd.setCursor(0, 2);
        lcd.print(str);
        break;
      }
      case INCH_DEC: {
        InchDecToString(pos, str);
        lcd.setCursor(0, 1);
        lcd.print(str);
        InchDecToString(pos - zero_pos, str);
        lcd.setCursor(0, 2);
        lcd.print(str);
        break;
      }
      case INCH_FRAC: {
        InchFracToString(pos, str);
        lcd.setCursor(0, 1);
        lcd.print(str);
        InchFracToString(pos - zero_pos, str);
        lcd.setCursor(0, 2);
        lcd.print(str);
        break;
      }
    }
    // Increments
    lcd.setCursor(0, 0);
    strcpy(str, "[------------------]");
    int32_t cursor = current_pos_inc % 18;
    if (cursor < 0) cursor += 18;
    str[cursor + 1] = 'O';
    lcd.print(str);
    lcd.setCursor(0, 3);
    lcd.print("\06/\05   \01      \02    \03\04");
  }  
}
