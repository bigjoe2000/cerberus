#include <Servo.h>

#define PIN_SERVO 9

Servo motor;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  motor.attach(PIN_SERVO);
  
}

int speeds[] = {65, 75, 85, 90, 95, 100, 110};

void loop() {
  int max = sizeof(speeds)/sizeof(int);
  for (int i = 0; i < max; i++) {
    motor.write(speeds[i]);
    delay(1000);
  }
}
