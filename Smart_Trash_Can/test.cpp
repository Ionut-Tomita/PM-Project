#include <Wire.h>
#include <Servo.h>
#include <Adafruit_ADXL345_U.h>

// === Devices ===
Servo myServo;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// === Pins ===
const int PIR_PIN = 2;
const int SERVO_PIN = 9;
const int BUZZER_PIN = 10;

// === Servo Angles ===
const int OPEN_ANGLE = 90;
const int CLOSED_ANGLE = 0;

// === Timings (ms) ===
const unsigned long DELAY_AFTER_OPEN = 3000;
const unsigned long ACCEL_INTERVAL = 500;

// === Alarm Config (Ambulance-style WAIL) ===
const unsigned long ALARM_DURATION = 4000;
const unsigned long ALARM_STEP_DELAY = 50;
const int ALARM_MIN_FREQ = 650;
const int ALARM_MAX_FREQ = 1300;
const int ALARM_STEP_FREQ = 25;

// === State tracking ===
enum { WAIT_FOR_MOTION, WAIT_TO_CLOSE, WAIT_FOR_RESET } state = WAIT_FOR_MOTION;
unsigned long openTimestamp = 0;
unsigned long lastAccelRead = 0;

// === Alarm state ===
bool alarmActive = false;
unsigned long alarmStart = 0;
unsigned long lastAlarmStep = 0;
int alarmFreq = ALARM_MIN_FREQ;
bool alarmRising = true;

// === Cooldown state ===
bool cooldown = false;
unsigned long cooldownStart = 0;
const unsigned long cooldownTime = 2000;

// === PIR motion flag ===
volatile bool pirTriggered = false;
volatile bool pirState = false;

void IRAM_ATTR pirISR() {
  pirTriggered = true;
  pirState = digitalRead(PIR_PIN);  // citim starea reală pentru control
}

// === Function: Update alarm siren ===
void updateAlarm(unsigned long now) {
  if (!alarmActive) return;

  if (now - alarmStart >= ALARM_DURATION) {
    noTone(BUZZER_PIN);
    alarmActive = false;
    return;
  }

  if (now - lastAlarmStep >= ALARM_STEP_DELAY) {
    tone(BUZZER_PIN, alarmFreq);
    lastAlarmStep = now;

    if (alarmRising) {
      alarmFreq += ALARM_STEP_FREQ;
      if (alarmFreq >= ALARM_MAX_FREQ) {
        alarmFreq = ALARM_MAX_FREQ;
        alarmRising = false;
      }
    } else {
      alarmFreq -= ALARM_STEP_FREQ;
      if (alarmFreq <= ALARM_MIN_FREQ) {
        alarmFreq = ALARM_MIN_FREQ;
        alarmRising = true;
      }
    }
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Booting...");

  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  myServo.attach(SERVO_PIN);
  myServo.write(CLOSED_ANGLE);
  Serial.println("Servo initialized");

  if (!accel.begin()) {
    Serial.println("ADXL345 not detected");
    while (1);
  }

  accel.setRange(ADXL345_RANGE_16_G);
  Serial.println("Smart bin ready");

  attachInterrupt(digitalPinToInterrupt(PIR_PIN), pirISR, CHANGE);  // detectăm atât RISING cât și FALLING
}

void loop() {
  unsigned long now = millis();

  // === PIR motion + servo control with interrupt ===
  static bool lastPirState = false;

  if (pirTriggered) {
    pirTriggered = false;

    if (pirState && state == WAIT_FOR_MOTION) {
      Serial.println("Motion detected! Opening bin…");
      myServo.write(OPEN_ANGLE);
      openTimestamp = now;
      state = WAIT_TO_CLOSE;
    }

    if (!pirState && state == WAIT_FOR_RESET) {
      Serial.println("PIR reset. Waiting for next motion.");
      state = WAIT_FOR_MOTION;
    }

    // Nu facem nimic dacă suntem în WAIT_TO_CLOSE – închidem automat mai jos
  }

  if (state == WAIT_TO_CLOSE && (now - openTimestamp >= DELAY_AFTER_OPEN)) {
    Serial.println("Time passed. Closing bin.");
    myServo.write(CLOSED_ANGLE);
    state = WAIT_FOR_RESET;
  }

  // === Accelerometer motion detection ===
  static float lastX = 0, lastY = 0, lastZ = 0;

  if (now - lastAccelRead >= ACCEL_INTERVAL) {
    lastAccelRead = now;

    sensors_event_t event;
    accel.getEvent(&event);

    float x = event.acceleration.x;
    float y = event.acceleration.y;
    float z = event.acceleration.z;

    Serial.print("Accel -> X: "); Serial.print(x);
    Serial.print(" Y: "); Serial.print(y);
    Serial.print(" Z: "); Serial.println(z);

    float dx = abs(x - lastX);
    float dy = abs(y - lastY);
    float dz = abs(z - lastZ);
    float totalDelta = dx + dy + dz;

    lastX = x;
    lastY = y;
    lastZ = z;

    if (!alarmActive && !cooldown && totalDelta > 12.0) {
      Serial.println("🚨 Strong movement detected! Triggering ambulance alarm.");
      alarmActive = true;
      alarmStart = now;
      lastAlarmStep = now;
      alarmFreq = ALARM_MIN_FREQ;
      alarmRising = true;
      cooldown = true;
      cooldownStart = now;
    }
  }

  // === Update alarm sweep if active
  updateAlarm(now);

  // === Reset cooldown after quiet time
  if (cooldown && (now - cooldownStart >= cooldownTime)) {
    cooldown = false;
  }
}