#include <FastAccelStepper.h>

#define ENA_PIN 4
#define DIR_PIN 3
#define PUL_PIN 9

// Stepper parameters
const uint32_t kMaxSpeed = 2000; // ticks/s
const uint32_t kMaxAccel = 10000; // ticks/s/s

// Time before we go to sleep, in ms
const uint32_t kSleepTime = 600000;

// The stepper motor controller object
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Number of ticks per revolutions
const int kTicksPerRev = 400;

// The value we load in timer1_counter everytime we get interrupted
#define TIMER1_VAL 65529   // 65536 - (16MHz/256 * 100us)

void setup() {
  Serial.begin(115200);

  // Stepper setup
  engine.init();
  stepper = engine.stepperConnectToPin(PUL_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENA_PIN);
    stepper->setAutoEnable(true);
    stepper->setSpeedInUs(200);  // the parameter is us/step !!!
    stepper->setAcceleration(5000);
    stepper->setDelayToEnable(0);
    stepper->setDelayToDisable(3000);
    stepper->enableOutputs();
    Serial.println("INIT DONE");
  } else {
    Serial.println("INIT FAILED");
  }  
  
  
//  stepper.setMaxSpeed(kMaxSpeed);
//  stepper.setAcceleration(kMaxAccel);
//  stepper.moveTo(0);

}



void loop() {
  uint32_t now = millis();
  uint8_t c;

  if (Serial.available() > 0) {
    for (int i = 0; i < Serial.available(); i++) {
      c = Serial.read();
    }
    switch (c) {
      case ']':
        if (digitalRead(ENA_PIN)) {
          stepper->enableOutputs();
          delay(2000);
        }
        stepper->move(400);
        break;
      case '[':
        if (digitalRead(ENA_PIN)) {
          stepper->enableOutputs();
          delay(2000);
        }
        stepper->move(-400);
        break;
    }
  }

}
