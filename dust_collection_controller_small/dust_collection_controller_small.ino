#include <Servo.h>


#define REMOTE_PIN 2
#define GATE_MITERSAW_PIN 3
#define GATE_BANDSAW_PIN 4
#define GATE_REEL_PIN 5
#define FAN_PIN 6
#define GPIO1_PIN 7
#define GPIO2_PIN 8
#define GPIO3_PIN 9
#define GPIO4_PIN 10
#define GPIO5_PIN 11
#define CURRENT_MITERSAW_PIN A0
#define CURRENT_SPARE0_PIN A1
#define CURRENT_BANDSAW_PIN A2
#define CURRENT_SPARE1_PIN A3

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
static const uint32_t kMaxFanOnTime = 3600000;

Servo servo_mitersaw;
Servo servo_bandsaw;
Servo servo_reel;

float current_mean_mitersaw  = 0.;
float current_mean_bandsaw   = 0.;
float current_mean2_mitersaw = 0.;
float current_mean2_bandsaw  = 0.;
uint32_t last_t = 0;
uint32_t last_t_saws_were_off = 0;
uint32_t last_t_saws_were_on  = 0;
uint32_t last_t_fan_was_off = 0;
bool last_mitersaw_on = false;
bool last_bandsaw_on = false;
bool last_reel_on = false;


void setup() {
  Serial.begin(115200);
  servo_mitersaw.attach(GATE_MITERSAW_PIN);
  servo_bandsaw.attach(GATE_BANDSAW_PIN);
  servo_reel.attach(GATE_REEL_PIN);
  servo_mitersaw.write(CLOSED);
  servo_bandsaw.write(CLOSED);
  servo_reel.write(CLOSED);
  pinMode(REMOTE_PIN, INPUT); 
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
  
  float current_mitersaw = ReadCurrent(CURRENT_MITERSAW_PIN);
  float current_bandsaw  = ReadCurrent(CURRENT_BANDSAW_PIN);

  // We want to compute the RMS of each signal.
  // If we estimate mean(I) and mean(I^2), we can compute the unbiased mean of squares with:
  //  (RMS(I))^2 = mean(I^2) - mean(I)^2
  // To keep a running average of the signals, we use an exponential smoothing with a given time constant
  // We use floating point math here. Slow, but we don't need to run that fast anyway
  float current_ms_mitersaw = ComputeMeanSquare(current_mitersaw,current_mean_mitersaw, current_mean2_mitersaw, t, last_t);
  float current_ms_bandsaw = ComputeMeanSquare(current_bandsaw,current_mean_bandsaw, current_mean2_bandsaw, t, last_t);

  // Threshold currents
  bool mitersaw_on = (current_ms_mitersaw > kCurrentThreshold * kCurrentThreshold);
  bool bandsaw_on = (current_ms_bandsaw > kCurrentThreshold * kCurrentThreshold);
  bool reel_on = digitalRead(REMOTE_PIN);
  bool any_saw_on = mitersaw_on || bandsaw_on;
  if (any_saw_on) {
    last_t_saws_were_on = t;
  } else {    
    last_t_saws_were_off = t;
  }

  // Turn the fan on and off with delays (or no delay if this comes from the reel)
  if (any_saw_on && (t > last_t_saws_were_off + kTurnOnDelayMs)) {
    digitalWrite(FAN_PIN, 1);
  }
  if (!any_saw_on && (t > last_t_saws_were_on + kTurnOffDelayMs)) {
    digitalWrite(FAN_PIN, 0);
  }
  if (reel_on) {
    digitalWrite(FAN_PIN, 1);  
  }

  // If the fan has been on for more than 1 hour, there is probably a problem. Turn it off.
  if (!digitalRead(FAN_PIN)) {
    last_t_fan_was_off = t;
  }

  if (t - last_t_fan_was_off > kMaxFanOnTime) {
    digitalWrite(FAN_PIN, 0);
  }

  // Manage the gates.
  if ((mitersaw_on != last_mitersaw_on) ||
      (bandsaw_on != last_bandsaw_on) ||
      (reel_on != last_reel_on)) {
    //  When the last machine turns off we want to keep its gate open, as
    //  it is likely that the same machine is going to turn on again soon.
    //  This will minimize the wear on the gate. 
    if (mitersaw_on || bandsaw_on || reel_on) {
      servo_mitersaw.write(mitersaw_on ? OPEN : CLOSED);
      servo_bandsaw.write(bandsaw_on ? OPEN : CLOSED);
      servo_reel.write(reel_on ? OPEN : CLOSED);
    }
  }

  last_mitersaw_on = mitersaw_on;
  last_bandsaw_on = bandsaw_on;
  last_reel_on = reel_on;
  last_t = t;
}
