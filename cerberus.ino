#include <NewPing.h>

#define MAX_DISTANCE 200

// Define which pins each of our sensors and actuators are connected to
#define PIN_PING_TRIGGER_RIGHT 3
#define PIN_PING_ECHO_RIGHT 4
#define PIN_PING_TRIGGER_LEFT 5
#define PIN_PING_ECHO_LEFT 6
#define PIN_LED 8
#define PIN_SERVO_LEFT 9
#define PIN_SERVO_RIGHT 10
#define PIN_BUZZER 11
#define PIN_CDS A0

bool wakeup_from_sensors;
unsigned long time_sleeping, next_ping_at, sleep_until, shine_until, next_breathe_at;
int current_distance_l, current_distance_r, shine_brightness, light_level, light_delta;

#define PING_SAMPLES 5
#define PING_SAMPLE_DELAY_MS 50

#define SNORE_DELAY_SECS 15
const int SNORE_DELAY_MS = SNORE_DELAY_SECS * 1000;
#define BREATHE_MIN 30
#define BREATHE_MAX 150
#define BREATHE_STEP 10
#define BREATHE_STEP_DUR_MS 50
int breathe_dir;

// Imported from https://github.com/raygeeknyc/photovore/blob/master/photovore.ino

#define sensorRPin 3
#define speakerPin 4
#define servoLPin PIN_SERVO_LEFT
#define servoRPin PIN_SERVO_RIGHT

// Define these based on your servos and controller, the values to cause your servos
// to spin in opposite directions at approx the same speed.
#define CW 30
#define CCW 10

#define SERVO_L_FWD CW
#define SERVO_R_FWD CCW

#define SERVO_L_BWD CCW
#define SERVO_R_BWD CW

#define SERVO_L_STOP 0
#define SERVO_R_STOP 0

#define SENSOR_DELTA_THRESHOLD_PCT 20
#define SENSOR_DELTA_THRESHOLD 30

#define HIGHEST_THRESHOLD SENSOR_DELTA_THRESHOLD

#define SENSOR_SAMPLES 5

#define MAX_SENSOR_READING 1023  // Used to seed sensor pair normalization

// How long to spin while callibrating the sensor pair
#define CALLIBRATION_DUR_MS 1000

// How long to pause between steps while spinning to normalize the sensor pair
#define SPIN_STEP_DELAY_MS 15
#define DIR_STOP 0
#define DIR_RIGHT 1
#define DIR_LEFT 2
#define DIR_FWD 3
#define LED_FLASH_DURATION_MS 500

NewPing sonarL(PIN_PING_TRIGGER_LEFT, PIN_PING_ECHO_LEFT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarR(PIN_PING_TRIGGER_RIGHT, PIN_PING_ECHO_RIGHT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

int current_dir, last_dir;
int sensor_normalization_delta;

void setup() {
  next_breathe_at = 0L;
  breathe_dir = 1;
  shine_until = 0L;
  sleep_until = 0L;
  next_ping_at = 0L;
  shine_brightness = 0;
  pinMode(servoLPin, OUTPUT);
  pinMode(servoRPin, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  analogWrite(servoLPin, SERVO_L_STOP);
  analogWrite(servoRPin, SERVO_R_STOP);
  sensor_normalization_delta = 0;
}

void recordDirection(int dir) {
  last_dir = current_dir;
  current_dir = dir;
}

int smooth(int array[], int len) {
  /**
    Return the average of the array without the highest and lowest values.
  **/
  int low = MAX_SENSOR_READING;
  int high = -1;
  int total = 0;
  for (int s = 0; s < len; s++) {
    total += array[s];
    low = min(array[s], low);
    high = max(array[s], high);
  }
  total -= low;
  total -= high;
  return total / (len - 2);
}

void drive(int direction) {
  recordDirection(direction);
  switch (direction) {
    case DIR_LEFT:
      analogWrite(servoLPin, SERVO_L_STOP);
      analogWrite(servoRPin, SERVO_R_FWD);
      break;
    case DIR_RIGHT:
      analogWrite(servoLPin, SERVO_L_FWD);
      analogWrite(servoRPin, SERVO_R_STOP);
      break;
    case DIR_FWD:
      analogWrite(servoLPin, SERVO_L_FWD);
      analogWrite(servoRPin, SERVO_R_FWD);
      break;
    case DIR_STOP:
      analogWrite(servoLPin, SERVO_L_STOP);
      analogWrite(servoRPin, SERVO_R_STOP);
      break;
  }
}

void readSensors() {  // raygeeknyc@
  // Don't read the ping sensors too often
  if (next_ping_at > millis()) {
    #ifdef _DEBUG
    Serial.print("Reuse old  distances. ");
    Serial.print("right: ");
    Serial.print(current_distance_r);
    Serial.print("left: ");
    Serial.println(current_distance_l);
    #endif
  } else {
    current_distance_l = getLeftPing();
    current_distance_r = getRightPing();
    next_ping_at = millis() + PING_SAMPLE_DELAY_MS;
  }
  int l = getLightLevel();
  light_delta = light_level - l;
  light_level = l;
}

int getLightLevel() {
   // Return the median reading from the light sensor
  int samples[SENSOR_SAMPLES];
  for (int sample = 0; sample < SENSOR_SAMPLES; sample++) {
    samples[sample] = analogRead(PIN_CDS);
  }
  return smooth(samples, SENSOR_SAMPLES);
}

/* Return true if we are currently sleeping, false if we're awake */
bool isSleeping() {  // raygeeknyc@
  return (sleep_until && sleep_until < millis());
}

void sleep(const unsigned sleep_duration_secs) {
  sleep_until = millis() + (sleep_duration_secs * 1000);
}

void spin(int direction) {
  recordDirection(direction);
  switch (direction) {
    case DIR_LEFT:
      analogWrite(servoLPin, SERVO_L_BWD);
      analogWrite(servoRPin, SERVO_R_FWD);
      break;
    case DIR_RIGHT:
      analogWrite(servoLPin, SERVO_L_FWD);
      analogWrite(servoRPin, SERVO_R_BWD);
      break;
    case DIR_STOP:
      analogWrite(servoLPin, SERVO_L_STOP);
      analogWrite(servoRPin, SERVO_R_STOP);
      break;
  }
}

/* Take the current step in moving about */
void roam() {
}

/* Pulse the LED in sleep mode */
void breathe() {  // raygeeknyc@
  if (!next_breathe_at || (next_breathe_at >= millis())) {
    next_breathe_at = millis() + BREATHE_STEP_DUR_MS;
  }
  if (next_breathe_at < millis()) {
    shine_brightness += BREATHE_STEP * ((breathe_dir > 0) ? 1 : -1);
    if ((shine_brightness <= BREATHE_MIN) || (shine_brightness >= BREATHE_MAX)) {
      breathe_dir *= -1;
    }
  }
}

/* Wake up, set flag, maybe make a waking noise or flash the LED */
void awaken() {  // raygeeknyc@
  sleep_until = 0l;
  next_breathe_at = 0;
  flashLed();
  burp();
}

bool isShining() {
  return (shine_until && shine_until < millis());
}

void flashLed() {
  shine_brightness = 255;
  shine_until = millis() + LED_FLASH_DURATION_MS;
}

void updateLed() {
  analogWrite(PIN_LED, shine_brightness);
}

/* Make a sleeping sound in sleep mode.
  Since this function blocks, update the breathing state LED */
void snore() {  // raygeeknyc@
  for (int i = 0; i < 6; i++) {
    beep(speakerPin, 125, 75);
    breathe();
    updateLed();
    beep(speakerPin, 75, 75);
    breathe();
    updateLed();
  }
}

// The sound producing function for chips without tone() support
void beep (unsigned char pin, int frequencyInHertz, long timeInMilliseconds) {
  // from http://web.media.mit.edu/~leah/LilyPad/07_sound_code.html
  int x;
  long delayAmount = (long)(1000000 / frequencyInHertz);
  long loopTime = (long)((timeInMilliseconds * 1000) / (delayAmount * 2));
  for (x = 0; x < loopTime; x++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(delayAmount);
    digitalWrite(pin, LOW);
    delayMicroseconds(delayAmount);
  }
}

// Emit a fairly rude noise
void burp() {
  beep(speakerPin, 125, 50);
  beep(speakerPin, 250, 75);
}

// Melody (liberated from the toneMelody Arduino example sketch by Tom Igoe).
int melody[] = { 262, 196, 196, 220, 196, 0, 247, 262 };
int duration[] = { 250, 125, 125, 250, 250, 250, 250, 250 };

void playTune() {
  for (int thisNote = 0; thisNote < 8; thisNote++) { // Loop through the notes in the array.
    beep(PIN_BUZZER, melody[thisNote], duration[thisNote]); // Play melody[thisNote] for duration[thisNote].
    delay(50); // Short delay between notes.
  }
}

int getLeftPing() {
  return getPingSensorReading(sonarL);
}

int getRightPing() {
  return getPingSensorReading(sonarR);
}

int getPingSensorReading(NewPing sonar) {
  int cm = 0;
  while (cm == 0) {
    int echoTime = sonar.ping_median(PING_SAMPLES);
    cm = sonar.convert_cm(echoTime);
  }
#ifdef _DEBUG
  Serial.print("Distance ");
  Serial.println(cm);
#endif
  return cm;
}

void loop() {
  readSensors();  // raygeeknyc@
  updateLed();  // raygeeknyc@ : done
  if (!isSleeping()) {  // raygeeknyc@ : done
    roam();  // raygeeknyc@
  }
  if (isSleeping()) {
    breathe();  // raygeeknyc@ : done
    if (wakeup_from_sensors) {
      awaken();  // raygeeknyc@ : done
    } else {
      if (time_sleeping > SNORE_DELAY_MS) {
        snore();  // raygeeknyc@ : done
      }
    }
  }
}
