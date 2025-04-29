#include <Servo.h>

#define FAN_PIN 2
#define GATE_SANDER_PIN 3
#define GATE_PLANER_PIN 4
#define GATE_ROUTER_PIN 5
#define GATE_JOINTER_PIN 6
#define GATE_SAW_PIN 7
#define GPIO0_PIN 8
#define GPIO1_PIN 9
#define GPIO2_PIN 10
#define GPIO3_PIN 11
#define GPIO4_PIN 12
#define GPIO5_PIN 13
#define CURRENT_PLANER_PIN A0
#define CURRENT_JOINTER_PIN A1
#define CURRENT_SPARE_PIN A2
#define CURRENT_SAW_PIN A3
#define CURRENT_SANDER_PIN A4
#define CURRENT_ROUTER_PIN A5

#define CLOSED 120
#define OPEN 0

static const float kResistanceOhm = 500;
static const float kXmerRatio = 1000;
static const float kAdcRangeV = 5;
static const float kAdcMaxVal = 1024;
static const float kAdcToAmps = kAdcRangeV / kAdcMaxVal / kResistanceOhm * kXmerRatio;
static const float kTimeConstantMs = 300.;
static const float kCurrentThreshold = 0.5;
static const uint32_t kTurnOnDelayMs = 1000;
static const uint32_t kTurnOffDelayMs = 5000;

Servo servo_sander;
Servo servo_planer;
Servo servo_router;
Servo servo_jointer;
Servo servo_saw;

float current_mean_saw      = 0.;
float current_mean_planer   = 0.;
float current_mean_jointer  = 0.;
float current_mean_sander   = 0.;
float current_mean_router   = 0.;
float current_mean2_saw     = 0.;
float current_mean2_planer  = 0.;
float current_mean2_jointer = 0.;
float current_mean2_sander  = 0.;
float current_mean2_router  = 0.;
uint32_t last_t = 0;
uint32_t last_t_machines_were_off = 0;
uint32_t last_t_machines_were_on  = 0;
bool last_saw_on = false;
bool last_router_on = false;
bool last_jointer_on = false;
bool last_planer_on = false;
bool last_sander_on = false;


void setup() {
  Serial.begin(115200);
  servo_sander.attach(GATE_SANDER_PIN);
  servo_planer.attach(GATE_PLANER_PIN);
  servo_router.attach(GATE_ROUTER_PIN);
  servo_jointer.attach(GATE_JOINTER_PIN);
  servo_saw.attach(GATE_SAW_PIN);
  servo_sander.write(CLOSED);
  servo_planer.write(CLOSED);
  servo_router.write(CLOSED);
  servo_jointer.write(CLOSED);
  servo_saw.write(CLOSED);
  pinMode(FAN_PIN, OUTPUT); 
}

float ReadCurrent(uint8_t pin) {
  uint16_t adc_val = analogRead(pin);
  return adc_val * kAdcToAmps;
}

float ComputeMeanSquare(float new_val, float& mean, float& mean2, uint32_t t, uint32_t last_t) {
  // On the first round, just use the values straight away
  if (!last_t) {
    mean = new_val;
    mean2 = new_val * new_val;
  } else {
    // Otherwise, use exponential smoothing to estimate the means
    float alpha = (t - last_t) / kTimeConstantMs;
    if (alpha > 1.0) {
      alpha = 1.0;
    }
    mean  = new_val * alpha + mean * (1.0 - alpha);
    mean2 = new_val * new_val * alpha + mean2 * (1.0 - alpha);
  }
  return mean2 - mean * mean;
}

void loop() {
  uint32_t t = millis();
  float current_saw     = ReadCurrent(CURRENT_SAW_PIN);
  float current_jointer = ReadCurrent(CURRENT_JOINTER_PIN);
  float current_planer  = ReadCurrent(CURRENT_PLANER_PIN);
  float current_router  = ReadCurrent(CURRENT_ROUTER_PIN);
  float current_sander  = ReadCurrent(CURRENT_SANDER_PIN);

  // We want to compute the RMS of each signal.
  // If we estimate mean(I) and mean(I^2), we can compute the unbiased mean of squares with:
  //  (RMS(I))^2 = mean(I^2) - mean(I)^2
  // To keep a running average of the signals, we use an exponential smoothing with a given time constant
  // We use floating point math here. Slow, but we don't need to run that fast anyway
  float current_ms_saw = ComputeMeanSquare(current_saw,current_mean_saw, current_mean2_saw, t, last_t);
  float current_ms_router = ComputeMeanSquare(current_router,current_mean_router, current_mean2_router, t, last_t);
  float current_ms_planer = ComputeMeanSquare(current_planer,current_mean_planer, current_mean2_planer, t, last_t);
  float current_ms_sander = ComputeMeanSquare(current_sander,current_mean_sander, current_mean2_sander, t, last_t);
  float current_ms_jointer = ComputeMeanSquare(current_jointer,current_mean_jointer, current_mean2_jointer, t, last_t);

  // Threshold currents
  bool saw_on = (current_ms_saw > kCurrentThreshold * kCurrentThreshold);
  bool router_on = (current_ms_router > kCurrentThreshold * kCurrentThreshold);
  bool planer_on = (current_ms_planer > kCurrentThreshold * kCurrentThreshold);
  bool jointer_on = (current_ms_jointer > kCurrentThreshold * kCurrentThreshold);
  bool sander_on = (current_ms_sander > kCurrentThreshold * kCurrentThreshold);
  bool any_machine_on = saw_on || router_on || planer_on || jointer_on || sander_on;
  if (any_machine_on) {
    last_t_machines_were_on = t;
  } else {    
    last_t_machines_were_off = t;
  }

  // Turn the fan on and off with delays
  if (any_machine_on && (t > last_t_machines_were_off + kTurnOnDelayMs)) {
    digitalWrite(FAN_PIN, 1);
  }
  if (!any_machine_on && (t > last_t_machines_were_on + kTurnOffDelayMs)) {
    digitalWrite(FAN_PIN, 0);
  }

  // Manage the gates.
  if ((saw_on != last_saw_on) ||
      (router_on != last_router_on) ||
      (sander_on != last_sander_on) ||
      (jointer_on != last_jointer_on) ||
      (planer_on != last_planer_on)) {
    //  When the last machine turns off we want to keep its gate open, as
    //  it is likely that the same machine is going to turn on again soon.
    //  This will minimize the wear on the gate. 
    if (router_on || saw_on || sander_on || jointer_on || planer_on) {
      servo_saw.write(saw_on ? OPEN : CLOSED);
      servo_router.write(router_on ? OPEN : CLOSED);
      servo_sander.write(sander_on ? OPEN : CLOSED);
      servo_jointer.write(jointer_on ? OPEN : CLOSED);
      servo_planer.write(planer_on ? OPEN : CLOSED);
    }
  }

  last_saw_on = saw_on;
  last_sander_on = sander_on;
  last_jointer_on = jointer_on;
  last_planer_on = planer_on;
  last_router_on = router_on;
  last_t = t;
  return;
  Serial.print(t);
  Serial.print(" ");
  Serial.print(current_router);
  Serial.print(" ");
  Serial.print(current_mean_router);
  Serial.print(" ");
  Serial.print(current_ms_router);
  Serial.print("\n");
}
