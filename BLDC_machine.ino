/*
 * IoT BLDC Motor Control with Hall Sensor Diagnostics
 * - Real-time Hall sensor pulse monitoring
 * - RPM measurement
 * - Blynk app control
 * - LCD display
 * - PWM control through ESC
 * - Hall sensor diagnostics
 *
 * Wiring:
 * ESC Signal  : D6
 * Hall Sensor : D5 (INPUT_PULLUP)
 * LCD I2C     : SDA=D2, SCL=D1
 */

#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "bldc motor"
#define BLYNK_AUTH_TOKEN "YOUR_BLYNK_AUTH_TOKEN"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <Ticker.h>
#include <LiquidCrystal_I2C.h>

// ================= WIFI =================
char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";

// ================= PIN DEFINITIONS =================
const uint8_t escPin = D6;
const uint8_t hallPin = D5;

// ================= BLYNK VIRTUAL PINS =================
#define VPIN_SETPOINT_RPM  V0
#define VPIN_ACTUAL_RPM    V1
#define VPIN_MOTOR_ON      V2
#define VPIN_THROTTLE      V3
#define VPIN_HALL_STATUS   V4
#define VPIN_HALL_PULSES   V5
#define VPIN_TEST_MODE     V6

// ================= OBJECTS =================
Ticker escPWM;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================= GLOBAL VARIABLES =================
volatile unsigned long pwmPulseWidthUs = 1000;

volatile unsigned long hallPulseCount = 0;
volatile unsigned long lastHallTime = 0;
volatile unsigned long lastHallPulseTime = 0;

// Motor control variables
int desiredRPM = 0;
int throttlePercent = 0;
int actualRPM = 0;
bool motorEnabled = false;
bool hallSensorActive = false;

// Hall sensor diagnostics
unsigned long hallPulsesPerSecond = 0;
unsigned long lastPulseInterval = 0;

// Timing
unsigned long lastLCDUpdate = 0;
unsigned long lastBlynkUpdate = 0;
unsigned long lastHallCheck = 0;

// ================= PWM GENERATION =================
void sendESCPulse() {
  digitalWrite(escPin, HIGH);
  delayMicroseconds(pwmPulseWidthUs);
  digitalWrite(escPin, LOW);
}

// ================= HALL SENSOR INTERRUPT =================
ICACHE_RAM_ATTR void hallSensorISR() {

  unsigned long currentMicros = micros();

  // Calculate pulse interval
  if (lastHallPulseTime > 0) {
    lastPulseInterval = currentMicros - lastHallPulseTime;
  }

  lastHallPulseTime = currentMicros;
  hallPulseCount++;
  lastHallTime = millis();
}

// ================= RPM CALCULATION =================
void calculateRPM() {

  static unsigned long lastCalcTime = 0;
  static unsigned long lastPulseCount = 0;

  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastCalcTime;

  // Calculate every 500 ms
  if (deltaTime >= 500) {

    unsigned long pulses =
      hallPulseCount - lastPulseCount;

    // Pulses per second
    hallPulsesPerSecond =
      (pulses * 1000UL) / deltaTime;

    // Adjust according to your motor
    const int pulsesPerRevolution = 1;

    if (pulses > 0) {

      actualRPM =
        (pulses * 60000UL) /
        (deltaTime * pulsesPerRevolution);

      hallSensorActive = true;
    }

    lastPulseCount = hallPulseCount;
    lastCalcTime = currentTime;
  }

  // Timeout detection
  // Motor stopped or Hall sensor not working
  if (millis() - lastHallTime > 1000) {

    actualRPM = 0;
    hallPulsesPerSecond = 0;

    if (motorEnabled && throttlePercent > 0) {
      hallSensorActive = false;
    }
  }
}

// ================= HALL SENSOR CHECK =================
void checkHallSensor() {

  if (millis() - lastHallCheck >= 500) {

    lastHallCheck = millis();

    if (lastHallPulseTime > 0 &&
        millis() - lastHallTime < 1000) {

      hallSensorActive = true;

    } else if (!motorEnabled ||
               throttlePercent == 0) {

      hallSensorActive = false;
    }
  }
}

// ================= RPM TO THROTTLE =================
int rpmToThrottle(int targetRPM) {

  const int MIN_RPM = 0;
  const int MAX_RPM = 8000;

  int throttle =
    map(targetRPM,
        MIN_RPM,
        MAX_RPM,
        25,
        100);

  return constrain(throttle, 25, 100);
}

// ================= SET THROTTLE =================
void setThrottle(int percent) {

  // Motor OFF
  if (!motorEnabled) {

    pwmPulseWidthUs = 1000;
    throttlePercent = 0;

    return;
  }

  // Constrain input
  int requestedThrottle =
    constrain(percent, 0, 100);

  // Minimum throttle
  const int MIN_THROTTLE = 25;

  if (requestedThrottle > 0 &&
      requestedThrottle < MIN_THROTTLE) {

    throttlePercent = MIN_THROTTLE;

  } else {

    throttlePercent = requestedThrottle;
  }

  // Convert throttle percentage to ESC pulse width
  pwmPulseWidthUs =
    map(throttlePercent,
        0,
        100,
        1000,
        2000);
}

// ================= BLYNK MOTOR ON/OFF =================
BLYNK_WRITE(VPIN_MOTOR_ON) {

  motorEnabled = param.asInt();

  if (!motorEnabled) {

    setThrottle(0);
    hallSensorActive = false;

    return;
  }

  // Motor ON
  if (desiredRPM == 0) {

    setThrottle(0);

  } else {

    throttlePercent =
      rpmToThrottle(desiredRPM);

    setThrottle(throttlePercent);
  }
}

// ================= BLYNK RPM SETPOINT =================
BLYNK_WRITE(VPIN_SETPOINT_RPM) {

  desiredRPM = param.asInt();

  // If motor is ON, update throttle immediately
  if (motorEnabled) {

    if (desiredRPM == 0) {

      setThrottle(0);

    } else {

      throttlePercent =
        rpmToThrottle(desiredRPM);

      setThrottle(throttlePercent);
    }
  }
}

// ================= BLYNK TEST MODE =================
BLYNK_WRITE(VPIN_TEST_MODE) {

  int testPercent = param.asInt();

  if (!motorEnabled) {

    setThrottle(0);
    return;
  }

  testPercent =
    constrain(testPercent, 0, 100);

  setThrottle(testPercent);
}

// ================= BLYNK CONNECTED =================
BLYNK_CONNECTED() {

  Blynk.syncVirtual(VPIN_SETPOINT_RPM);
  Blynk.syncVirtual(VPIN_MOTOR_ON);
}

// ================= LCD UPDATE =================
void updateLCD() {

  // Line 1: Set RPM and Actual RPM
  lcd.setCursor(0, 0);

  lcd.print("S:");
  lcd.print(desiredRPM);

  if (desiredRPM < 10)
    lcd.print("   ");
  else if (desiredRPM < 100)
    lcd.print("  ");
  else if (desiredRPM < 1000)
    lcd.print(" ");

  lcd.print("A:");
  lcd.print(actualRPM);

  if (actualRPM < 10)
    lcd.print("   ");
  else if (actualRPM < 100)
    lcd.print("  ");
  else if (actualRPM < 1000)
    lcd.print(" ");

  // Line 2: Throttle and motor status
  lcd.setCursor(0, 1);

  lcd.print("T:");
  lcd.print(throttlePercent);

  if (throttlePercent < 10)
    lcd.print("% ");
  else if (throttlePercent < 100)
    lcd.print("%");
  else
    lcd.print("");

  lcd.print(" ");

  if (motorEnabled) {
    lcd.print("ON ");
  } else {
    lcd.print("OFF");
  }

  // Hall pulse information
  lcd.print(" H:");
  lcd.print(hallPulsesPerSecond);
}

// ================= SETUP =================
void setup() {

  Serial.begin(115200);

  // Initialize LCD
  Wire.begin(D2, D1);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("BLDC + Hall");

  lcd.setCursor(0, 1);
  lcd.print("Diagnostics v2.0");

  delay(2000);

  // Initialize pins
  pinMode(escPin, OUTPUT);
  pinMode(hallPin, INPUT_PULLUP);

  digitalWrite(escPin, LOW);

  // Initial ESC pulse
  pwmPulseWidthUs = 1000;

  // Start PWM signal at 50 Hz
  escPWM.attach_ms(20, sendESCPulse);

  delay(2000);

  // Attach Hall sensor interrupt
  attachInterrupt(
    digitalPinToInterrupt(hallPin),
    hallSensorISR,
    FALLING
  );

  // ================= WIFI =================
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi...");

  WiFi.begin(ssid, pass);

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED &&
         attempts < 30) {

    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("WiFi OK");

    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());

    delay(2000);

    // ================= BLYNK =================
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Blynk...");

    Blynk.config(BLYNK_AUTH_TOKEN);

    if (Blynk.connect()) {

      lcd.setCursor(0, 1);
      lcd.print("Connected!");

      delay(2000);

    } else {

      lcd.setCursor(0, 1);
      lcd.print("Failed!");

      delay(2000);
    }

  } else {

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed!");

    delay(3000);
  }

  // Final initialization
  pwmPulseWidthUs = 1000;
  throttlePercent = 0;
  desiredRPM = 0;
  motorEnabled = false;
  hallSensorActive = false;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");

  lcd.setCursor(0, 1);
  lcd.print("Motor: OFF");

  delay(2000);

  lcd.clear();
}

// ================= MAIN LOOP =================
void loop() {

  Blynk.run();

  // Calculate RPM from Hall sensor
  calculateRPM();

  // Check Hall sensor status
  checkHallSensor();

  // Update LCD every 200 ms
  if (millis() - lastLCDUpdate >= 200) {

    lastLCDUpdate = millis();

    updateLCD();
  }

  // Send data to Blynk every 500 ms
  if (millis() - lastBlynkUpdate >= 500) {

    lastBlynkUpdate = millis();

    Blynk.virtualWrite(
      VPIN_ACTUAL_RPM,
      actualRPM
    );

    Blynk.virtualWrite(
      VPIN_THROTTLE,
      throttlePercent
    );

    Blynk.virtualWrite(
      VPIN_HALL_STATUS,
      hallSensorActive ? "Active" : "Inactive"
    );

    Blynk.virtualWrite(
      VPIN_HALL_PULSES,
      hallPulseCount
    );
  }
}
