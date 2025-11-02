#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_ADXL345_U.h>
#include <LiquidCrystal_I2C.h>

// === Pini ===
const int PIR_PIN = 7;
const int SERVO_PIN = 9;
const int BUZZER_PIN = 8;
const int TRIG_PIN = 5;
const int ECHO_PIN = 6;

// === Servo motor ===
Servo myServo;
const int OPEN_ANGLE = 90;
const int CLOSED_ANGLE = 0;

// === Accelerometru ===
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
const float MOVEMENT_THRESHOLD = 30.0;

// === Temporizare capac ===
bool isOpen = false;
unsigned long openTimestamp = 0;
const unsigned long OPEN_DURATION = 3000;

// === Ultrasonic ===
unsigned long lastUltrasonicCheck = 0;
const unsigned long ULTRASONIC_INTERVAL = 300;
float lastDistance = 30.0;
int objectCount = 0;

// === LCD I2C ===
LiquidCrystal_I2C lcd(0x27, 16, 2);

// === Sirena buzzer ===
bool alarmActive = false;
unsigned long alarmStart = 0;
unsigned long lastAlarmStep = 0;

const unsigned long ALARM_DURATION = 4000;    // Durata sirenă (ms)
const unsigned long ALARM_STEP_DELAY = 30;    // Interval schimbare frecvență (ms)

int alarmFreq = 650;      // Frecvență inițială (Hz)
const int ALARM_MIN_FREQ = 650;
const int ALARM_MAX_FREQ = 1300;
const int ALARM_STEP_FREQ = 20;

bool alarmRising = true;

// Cooldown pentru sirenă
bool cooldown = false;
unsigned long cooldownStart = 0;
const unsigned long cooldownDuration = 2000;

// Pentru afișare licărindă
unsigned long lastBlink = 0;
const unsigned long BLINK_INTERVAL = 500;
bool blinkState = false;

// Pentru detectarea golirii coșului
bool binFull = false;
unsigned long handNearStart = 0;
const unsigned long HAND_NEAR_DURATION = 3000;
bool handNear = false;

void startAlarm(unsigned long now) {
  alarmActive = true;
  alarmStart = now;
  lastAlarmStep = now;
  alarmFreq = ALARM_MIN_FREQ;
  alarmRising = true;
  cooldown = true;
  cooldownStart = now;
  tone(BUZZER_PIN, alarmFreq);
}

void updateAlarm(unsigned long now) {
  if (!alarmActive) return;

  if (now - alarmStart >= ALARM_DURATION) {
    noTone(BUZZER_PIN);
    alarmActive = false;
    return;
  }

  if (now - lastAlarmStep >= ALARM_STEP_DELAY) {
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

    tone(BUZZER_PIN, alarmFreq);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  noTone(BUZZER_PIN);

  myServo.attach(SERVO_PIN);
  myServo.write(CLOSED_ANGLE);

  if (!accel.begin()) {
    Serial.println("Accelerometrul nu a fost detectat!");
    
  }
  accel.setRange(ADXL345_RANGE_16_G);

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();

  Serial.println("Sistem pornit!");
}

void loop() {
  unsigned long now = millis();

  // === Afișare Smart Bin Ready până apare primul obiect ===
  if (objectCount == 0 && !binFull) {
    lcd.setCursor(0, 0);
    lcd.print("Smart Bin Ready ");
  }

  // === PIR logic pentru capac ===
  bool motion = digitalRead(PIR_PIN);
  if (!isOpen && motion && !binFull) {
    Serial.println("Miscare detectata - Deschid capacul");
    myServo.write(OPEN_ANGLE);
    openTimestamp = now;
    isOpen = true;
  }

  if (isOpen && !binFull && (now - openTimestamp >= OPEN_DURATION)) {
    Serial.println("Timpul a expirat - Inchid capacul");
    myServo.write(CLOSED_ANGLE);
    isOpen = false;
  }

  // === Detectie accelerometru ===
  sensors_event_t event;
  accel.getEvent(&event);

  float dx = abs(event.acceleration.x);
  float dy = abs(event.acceleration.y);
  float dz = abs(event.acceleration.z);
  float totalDelta = dx + dy + dz;

  if (!cooldown && totalDelta > MOVEMENT_THRESHOLD) {
    Serial.println("⚠️ Miscare brusca detectata - Activare buzzer");
    startAlarm(now);
  }

  if (cooldown && (now - cooldownStart >= cooldownDuration)) {
    cooldown = false;
  }

  updateAlarm(now);

  // === Senzor ultrasonic: detectare obiecte ===
  if (now - lastUltrasonicCheck >= ULTRASONIC_INTERVAL) {
    lastUltrasonicCheck = now;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    float distance = duration * 0.034 / 2;

    Serial.print("Distanta: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Verifică dacă un obiect a fost introdus (scădere rapidă a distanței)
    if ((lastDistance - distance) > 5.0 && distance < 15.0 && !binFull) {
      objectCount++;
      Serial.print("Obiect detectat! Total: ");
      Serial.println(objectCount);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Obiecte in cos:");
      lcd.setCursor(0, 1);
      lcd.print(objectCount);

      delay(1000);  // debounce
    }

    lastDistance = distance;
  }

  // === Licărire mesaje de alertă când sirena e activă ===
  if (alarmActive) {
    if (blinkState == false) {
      lcd.clear();  // Sterge ecranul înainte de licărire
    }
    if (now - lastBlink >= BLINK_INTERVAL) {
      lastBlink = now;
      blinkState = !blinkState;
      lcd.setCursor(0, 0);
      if (blinkState) {
        lcd.print("Atentie!!!       ");
      } else {
        lcd.print("                 ");
      }
    }
  } else {
    // Când sirena nu mai e activă, afișează Smart Bin Ready dacă coșul nu e plin
    if (!binFull) {
      lcd.setCursor(0, 0);
      lcd.print("Smart Bin Ready ");
    }
  }


  // === Când coșul are 5 obiecte, licărire mesaj 'Cos de gunoi plin' și coș deschis ===
  if (objectCount >= 5) {
    binFull = true;
    myServo.write(OPEN_ANGLE);

    // Stergem o singura data inainte de licarire, cand blinkState este false
    if (blinkState == false) {
        lcd.clear();
    }

    if (now - lastBlink >= BLINK_INTERVAL) {
        lastBlink = now;
        blinkState = !blinkState;
        lcd.setCursor(0, 0);
        if (blinkState) {
            lcd.print("Cos plin");
        } else {
            lcd.print("                 ");
        }
    }

    // Verifică dacă mâna este aproape de senzor (sub 5 cm)
    if (lastDistance < 5.0) {
        if (!handNear) {
            handNear = true;
            handNearStart = now;
        } else {
            if (now - handNearStart >= HAND_NEAR_DURATION) {
                // Golire confirmată: resetare contor și stare coș
                objectCount = 0;
                binFull = false;
                myServo.write(CLOSED_ANGLE);
                lcd.clear();
                Serial.println("Cos golit, resetez...");
            }
        }
    } else {
        handNear = false;
        handNearStart = 0;
    }
  }

}
