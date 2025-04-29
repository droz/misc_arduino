#include <FastAccelStepper.h>
#include <LiquidCrystal_I2C.h>

#define ENA_PIN 8
#define DIR_PIN 7
#define PUL_PIN 9
#define PUSH0_PIN 4
#define PUSH1_PIN 3
#define LED0_PIN 6
#define LED1_PIN 5
#define LIMIT_PIN 2

// Stepper parameters
const uint32_t kMaxSpeed = 200;    // us/step
const uint32_t kMaxAccel = 20000;  // ticks/s/s

// Time before we go to sleep, in ms
const uint32_t kSleepTime = 600000;
// Time before we release the enable of the stepper, in ms
const uint32_t kStepperReleaseTime = 1000;

// The stepper motor controller object
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Number of ticks per revolutions
const int kTicksPerRev = 400;
// The pitch of the lead screw, in mm
const float kScrewPitchMm = 2;

// The conversion from steps to mm
const float kStepInMm = kScrewPitchMm / kTicksPerRev;

// The tick at which we saw the limit switch
int32_t zero_pos = 0;
// Have we seen the limit switch ?
bool seen_zero = false;

// The up and down limits
int32_t kMaxPos = 600;
// This is the absolute min possible pos: 
// int32_t kMinPos = -33600;
// And this is the one to end up right under the table:
int32_t kMinPos = -30000;

struct IOState {
  uint8_t pin;
  bool last_state;
  uint32_t last_change_time;
  bool pressed;
  bool released;
};

// Debouncer time constant (milliseconds)
static const uint32_t kDebounceTime = 100;

// IO state structures
IOState push0;
IOState push1;
IOState limit;

// Reset function (jump to address 0)
void(* resetFunc) (void) = 0;



void setup() {
  Serial.begin(115200);

  // Stepper setup
  engine.init();
  stepper = engine.stepperConnectToPin(PUL_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENA_PIN);
    stepper->setAutoEnable(true);
    stepper->setSpeedInUs(kMaxSpeed);
    stepper->setAcceleration(kMaxAccel);
    stepper->setDelayToEnable(0);
    stepper->setDelayToDisable(0);
    stepper->disableOutputs();
    Serial.println("INIT DONE");
  } else {
    Serial.println("INIT FAILED");
  } 

  // Buttons setup
  pinMode(PUSH0_PIN, INPUT_PULLUP); 
  pinMode(PUSH1_PIN, INPUT_PULLUP); 
  pinMode(LED0_PIN, OUTPUT); 
  pinMode(LED1_PIN, OUTPUT); 
  digitalWrite(LED0_PIN, HIGH);
  digitalWrite(LED1_PIN, HIGH);
  InitState(PUSH0_PIN, &push0);
  InitState(PUSH1_PIN, &push1);
  
  // Limit Switch
  pinMode(LIMIT_PIN, INPUT_PULLUP); 
  InitState(LIMIT_PIN, &limit);
}

void InitState(uint8_t pin, IOState* state) {
  state->last_change_time = 0;
  state->last_state = !digitalRead(pin);
  state->pressed = false;
  state->released = false;
  state->pin = pin;
}

void ReadAndDebounce(IOState* io) {
  uint32_t t = millis();
  io->pressed = false;
  io->released = false;
  // If the IO changed state recently, don't even sample it (debouncing).
  if (t < (io->last_change_time + kDebounceTime)) {
    return;
  }
  bool new_state = !digitalRead(io->pin);
  if (new_state == io->last_state) {
    return;
  }
  io->last_change_time = t;
  if (!io->last_state && new_state) {
    io->pressed = true;
  }
  if (io->last_state && !new_state) {
    io->released = true;
  }
  io->last_state = new_state;
}

// This function makes sure that we give enough time between enabling the stepper and moving
void StepperTurnOnAndMove(long relative) {
  if (digitalRead(ENA_PIN)) {
    stepper->enableOutputs();
    delay(100);
  }
  stepper->move(relative);
  Serial.print("Moving relative: ");
  Serial.print(relative);
  Serial.print("\n");
}
void StepperTurnOnAndMoveTo(long absolute) {
  if (digitalRead(ENA_PIN)) {
    stepper->enableOutputs();
    delay(100);
  }
  stepper->moveTo(absolute);
  Serial.print("Moving to position: ");
  Serial.print(absolute);
  Serial.print("\n");
}

// Wait for the stepper to be done moving
void WaitStopped() {
  while(stepper->isRunning()) {
    delay(1);
  }
}

void loop() {
  uint32_t now = millis();
  ReadAndDebounce(&push0);
  ReadAndDebounce(&push1);
  ReadAndDebounce(&limit);
  int32_t pos = stepper->getCurrentPosition();

  // When we see the limit switch, store the zero and change the target pos.
  if (limit.pressed) {
    zero_pos = pos;
    seen_zero = true;
    StepperTurnOnAndMoveTo(zero_pos + kMaxPos);
    Serial.print("Limit switch triggered at position: ");
    Serial.print(zero_pos);
    Serial.print("\n");
  }
  
  if (push0.pressed) {
    if (seen_zero) {
      StepperTurnOnAndMoveTo(zero_pos + kMinPos);
    } else {
      // In the case where we are past the limit switch, we should allow downward motion,
      // but only until the limit switch is un-triggered
      if (limit.last_state) {
        StepperTurnOnAndMove(-1000000);
      }
    }
    Serial.print("DOWN button was pressed\n");
  }
  if (push1.pressed) {
    if (seen_zero) {
      StepperTurnOnAndMoveTo(zero_pos + kMaxPos);
    } else {
      // If the limit switch is already triggered, don't allow motion.
      if (!limit.last_state) {
        StepperTurnOnAndMove(1000000);
      }
    }
    Serial.print("UP button was pressed\n");
  }
  if (push0.released || push1.released) {
    stepper->stopMove();
  }
  // If we started with the limit switch on and started by going down, we need to stop when
  //  the limit switch is un-triggered
  if (!seen_zero && limit.released) {
    stepper->stopMove();
  }

  // Turn LEDs on and off depending on the available directions
  bool led0 = true;
  bool led1 = true;
  if (seen_zero) {
    if (pos == zero_pos + kMaxPos) led0 = false;
    if (pos == zero_pos + kMinPos) led1 = false;
  } else {
    if (limit.last_state) {
      led0 = false;
    } else {
      led1 = false;
    }
  }
  digitalWrite(LED0_PIN, led1);
  digitalWrite(LED1_PIN, led0);
}
