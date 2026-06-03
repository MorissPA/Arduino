/*
 * GŁÓWNY TEST: Bluetooth + 2x L298N + LCD 16x2 I2C + HC-SR04 + Buzzer + LED + PCA9685
 *
 * HC-05:    TX->pin19, RX->pin18
 * L298N #1: ENA=5,  IN1=22, IN2=23, ENB=6,  IN3=24, IN4=25
 * L298N #2: ENA=7,  IN1=26, IN2=27, ENB=8,  IN3=28, IN4=29
 * LCD I2C:  SDA=pin20, SCL=pin21 (adres 0x27)
 * PCA9685:  SDA=pin20, SCL=pin21 (adres 0x40) — LEDy na kanałach 0-3
 * HC-SR04:  TRIG=38, ECHO=40
 * Buzzer:   Pin 44
 * LED:      Pin 46
 *
 * Wymagane biblioteki:
 *   - LiquidCrystal I2C (Frank de Brabander)
 *   - Adafruit PWM Servo Driver Library
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_PWMServoDriver.h>

// ----- LCD -----
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ----- PCA9685 -----
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
#define LED_COUNT 4       // kanały 0-3
#define PWM_MAX   4095    // 12-bit

// ----- HC-SR04 -----
#define TRIG_PIN  38
#define ECHO_PIN  40
#define STOP_DIST 20  // cm

// ----- BUZZER i LED -----
#define BUZZER_PIN 44
#define LED_PIN    46

// ----- L298N #1 -----
#define ENA 5
#define IN1 22
#define IN2 23
#define ENB 6
#define IN3 24
#define IN4 25

// ----- L298N #2 -----
#define ENC 7
#define IN5 26
#define IN6 27
#define END 8
#define IN7 28
#define IN8 29

// ----- ZMIENNE -----
int predkosc          = 200;
bool jedzie_przod     = false;
bool za_blisko        = false;
bool led_stan         = false;

unsigned long czas_pomiaru = 0;
unsigned long czas_led     = 0;
unsigned long czas_pulse   = 0;

long odleglosc_cm          = 999;
long prev_odleglosc        = -1;
char aktualny_kierunek[17] = "STOP";
char prev_kierunek[17]     = "";

// pulsowanie LED na PCA9685
int pulse_brightness = 0;
int pulse_krok       = 40;   // krok zmiany jasności (większy = szybsze pulsowanie)

// ----- SILNIKI -----
void silnikA(int8_t k) {
  if      (k > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (k < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else            { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW); }
  analogWrite(ENA, k != 0 ? predkosc : 0);
}

void silnikB(int8_t k) {
  if      (k > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (k < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else            { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW); }
  analogWrite(ENB, k != 0 ? predkosc : 0);
}

void silnikC(int8_t k) {
  if      (k > 0) { digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW); }
  else if (k < 0) { digitalWrite(IN5, LOW);  digitalWrite(IN6, HIGH); }
  else            { digitalWrite(IN5, LOW);  digitalWrite(IN6, LOW); }
  analogWrite(ENC, k != 0 ? predkosc : 0);
}

void silnikD(int8_t k) {
  if      (k > 0) { digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW); }
  else if (k < 0) { digitalWrite(IN7, LOW);  digitalWrite(IN8, HIGH); }
  else            { digitalWrite(IN7, LOW);  digitalWrite(IN8, LOW); }
  analogWrite(END, k != 0 ? predkosc : 0);
}

void stop_all() {
  silnikA(0); silnikB(0); silnikC(0); silnikD(0);
  jedzie_przod = false;
}

// ----- HC-SR04 -----
long zmierz_odleglosc() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long czas = pulseIn(ECHO_PIN, HIGH, 25000);
  if (czas == 0) return 999;
  return czas / 58;
}

// ----- LCD (tylko gdy zmiana) -----
void aktualizuj_lcd() {
  bool zmiana = false;

  if (strcmp(aktualny_kierunek, prev_kierunek) != 0) {
    lcd.setCursor(0, 0);
    lcd.print("Kier:           ");
    lcd.setCursor(6, 0);
    lcd.print(aktualny_kierunek);
    strcpy(prev_kierunek, aktualny_kierunek);
    zmiana = true;
  }

  if (odleglosc_cm != prev_odleglosc) {
    lcd.setCursor(0, 1);
    if (odleglosc_cm >= 999) {
      lcd.print("Odl: poza zasieg");
    } else {
      lcd.print("Odl: ");
      lcd.print(odleglosc_cm);
      lcd.print(" cm         ");
    }
    prev_odleglosc = odleglosc_cm;
    zmiana = true;
  }
}

void info(const char* txt) {
  Serial.println(txt);
  Serial1.println(txt);
}

// ----- SETUP -----
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(ENC, OUTPUT);
  pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT); pinMode(END, OUTPUT);
  stop_all();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // PCA9685
  pca.begin();
  pca.setPWMFreq(1000);  // 1 kHz dla LEDów
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    pca.setPin(i, 0);    // wyzeruj wszystkie kanały
  }

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  SAMOCHODZIK   ");
  lcd.setCursor(0, 1);
  lcd.print("   GOTOWY :)    ");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Kier: STOP");

  info("=== GOTOWY ===");
  info("F/B/L/R/S/+/-");
}

// ----- LOOP -----
void loop() {
  unsigned long teraz = millis();

  // --- pomiar odległości co 150ms ---
  if (teraz - czas_pomiaru >= 150) {
    czas_pomiaru = teraz;
    odleglosc_cm = zmierz_odleglosc();
    za_blisko = (odleglosc_cm < STOP_DIST);

    if (jedzie_przod && za_blisko) {
      stop_all();
      strcpy(aktualny_kierunek, "AUTO-STOP");
      tone(BUZZER_PIN, 1000, 300);
      info("AUTO-STOP!");
    }

    aktualizuj_lcd();
  }

  // --- mruganie LED (pin 46) gdy za blisko ---
  if (teraz - czas_led >= 200) {
    czas_led = teraz;
    if (za_blisko) {
      led_stan = !led_stan;
      digitalWrite(LED_PIN, led_stan);
      if (led_stan) tone(BUZZER_PIN, 1500, 100);
    } else {
      digitalWrite(LED_PIN, LOW);
      led_stan = false;
    }
  }

  // --- pulsowanie LEDów na PCA9685 co 20ms ---
  if (teraz - czas_pulse >= 20) {
    czas_pulse = teraz;

    pulse_brightness += pulse_krok;
    if (pulse_brightness >= PWM_MAX) {
      pulse_brightness = PWM_MAX;
      pulse_krok = -40;
    } else if (pulse_brightness <= 0) {
      pulse_brightness = 0;
      pulse_krok = 40;
    }

    for (uint8_t i = 0; i < LED_COUNT; i++) {
      pca.setPin(i, pulse_brightness);
    }
  }

  // --- odczyt komendy ---
  char cmd = 0;
  if (Serial.available())  cmd = Serial.read();
  if (Serial1.available()) cmd = Serial1.read();
  if (cmd == 0) return;

  switch (cmd) {
    case 'F': case 'f':
      if (!za_blisko) {
        silnikA(-1); silnikB(-1); silnikC(-1); silnikD(-1);
        jedzie_przod = true;
        strcpy(aktualny_kierunek, "PRZOD");
        info("PRZOD");
      } else {
        info("BLOKADA - za blisko!");
      }
      break;

    case 'B': case 'b':
      silnikA(1); silnikB(1); silnikC(1); silnikD(1);
      jedzie_przod = false;
      strcpy(aktualny_kierunek, "TYL");
      info("TYL");
      break;

    case 'L': case 'l':
      silnikA(1);  silnikC(1);
      silnikB(-1); silnikD(-1);
      jedzie_przod = false;
      strcpy(aktualny_kierunek, "LEWO");
      info("LEWO");
      break;

    case 'R': case 'r':
      silnikA(-1); silnikC(-1);
      silnikB(1);  silnikD(1);
      jedzie_przod = false;
      strcpy(aktualny_kierunek, "PRAWO");
      info("PRAWO");
      break;

    case 'S': case 's':
      stop_all();
      strcpy(aktualny_kierunek, "STOP");
      info("STOP");
      break;

    case '+':
      predkosc = min(255, predkosc + 20);
      Serial.print("Predkosc: "); Serial.println(predkosc);
      Serial1.print("Predkosc: "); Serial1.println(predkosc);
      break;

    case '-':
      predkosc = max(60, predkosc - 20);
      Serial.print("Predkosc: "); Serial.println(predkosc);
      Serial1.print("Predkosc: "); Serial1.println(predkosc);
      break;
  }
}