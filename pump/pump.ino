#include <FastAccelStepper.h>

#define ENA_PIN 8
#define DIR_PIN 7
#define PUL_PIN 9
#define CTRL_PIN A7

// The stepper motor controller object
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Stepper parameters
const uint32_t kNumSteps = 800;
const uint32_t kMaxSpeed = 2000;    // 1000*rev/s

void setup() {
  Serial.begin(115200);

  // Setup the stepper pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(ENA_PIN, LOW);
}

void loop() {
  uint32_t ctrl = analogRead(CTRL_PIN);
  if (ctrl < 30) {
    digitalWrite(ENA_PIN, HIGH);
    delay(10);
    return;
  }
  if (ctrl > 50) {
    digitalWrite(ENA_PIN, LOW);
  }
  digitalWrite(PUL_PIN, HIGH);
  digitalWrite(PUL_PIN, LOW);
  uint32_t speed = kMaxSpeed * ctrl / 1024;
  int32_t t = 1000000000 / (kNumSteps * speed);
  while(t > 0) {
    if (t > 16383) {
      delayMicroseconds(16383);
      t -= 16383;
    } else {
      delayMicroseconds(t);
      t = 0;
    }
  }
}
