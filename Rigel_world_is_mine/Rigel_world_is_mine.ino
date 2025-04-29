#include <Servo.h>


#define LASER_PIN 2
#define SERVO1_PIN 4
#define SERVO2_PIN 6

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  pinMode(LASER_PIN, OUTPUT);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
}

void loop() {
  digitalWrite(LASER_PIN, 1);

   
  servo1.write(120 + random(-20, 20));
  servo2.write(110 + random(-20, 20));
  delay(random(10, 1000));
}
