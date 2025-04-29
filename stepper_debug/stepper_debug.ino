#include <AccelStepper.h>

#define ENA_PIN 4
#define DIR_PIN 3
#define PUL_PIN 9

// Stepper parameters
const uint32_t kMaxSpeed = 2000; // ticks/s
const uint32_t kMaxAccel = 10000; // ticks/s/s

// Time before we go to sleep, in ms
const uint32_t kSleepTime = 600000;

// The stepper motor controller object
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN, 0, 0);

// Number of ticks per revolutions
const int kTicksPerRev = 400;

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

  // We need to call the steppers run() function at a minimum of 3kHz.
  // We are going to setup
  // a timer interrupt running at 10kHz to take care of it.
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = TIMER1_VAL;       // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts 
}

// Timer 1 interrupt routine
ISR(TIMER1_OVF_vect)
{
  TCNT1 = TIMER1_VAL;   // preload timer
  stepper.run();
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
void StepperSetMaxSpeed(long speed) {
  noInterrupts();
  stepper.setMaxSpeed(speed);
  interrupts();
}

void loop() {
  uint32_t now = millis();
  uint8_t c;

  if (Serial.available() > 0) {
    c = Serial.read();
    switch (c) {
      case ']':
        StepperMove(400);
        break;
      case '[':
        StepperMove(-400);
        break;
      case '1':
        StepperSetMaxSpeed(1000);
        break;
      case '2':
        StepperSetMaxSpeed(1500);
        break;
      case '3':
        StepperSetMaxSpeed(2000);
        break;
      case '4':
        StepperSetMaxSpeed(2500);
        break;
      case '5':
        StepperSetMaxSpeed(3000);
        break;
      case '6':
        StepperSetMaxSpeed(3500);
        break;
      case '7':
        StepperSetMaxSpeed(4000);
        break;
      case '8':
        StepperSetMaxSpeed(4500);
        break;
      case '9':
        StepperSetMaxSpeed(5000);
        break;
      case '0':
        StepperSetMaxSpeed(5500);
        break;

    }
  }

}
