#include <Arduino.h>
#include <IRremote.hpp>
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include "time.h"
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

constexpr uint8_t IR_RECEIVE_PIN = 14;
constexpr uint8_t BUZZER_PIN = 25;
constexpr uint8_t trigPin = 18;
constexpr uint8_t echoPin = 19;
constexpr uint8_t IRsensor = 23;

constexpr unsigned long SERIAL_BAUD = 9600;
constexpr decode_type_t REMOTE_PROTOCOL = NEC;
constexpr uint16_t REMOTE_ADDRESS = 0x0;
constexpr uint16_t POWER_BUTTON_COMMAND = 0x45;
constexpr uint16_t RESET_BUTTON_COMMAND = 0x44;

constexpr float threshold = 10.0f;
constexpr unsigned long SONIC_TIMEOUT_US = 26000;
constexpr unsigned long EVENT_COOLDOWN_MS = 2000;
constexpr unsigned long BUZZER_ON_TIME_MS = 5000;
constexpr unsigned long STATUS_PRINT_INTERVAL_MS = 1000;

bool power = false;
bool alarmActive = false;
bool isIRActivated = false;
bool isSonicActivated = false;
bool lastIRStateHigh = false;

int peopleInCount = 0;
int peopleOutCount = 0;
int numPeople = 0;

long duration = 0;
float previousCm = 0.0f;

unsigned long IR_ActivationTime = 0;
unsigned long Ultra_ActivationTime = 0;
unsigned long lastEventTime = 0;
unsigned long buzzerStartedAt = 0;
unsigned long lastStatusPrintAt = 0;

const char* ssid = "NewHome";
const char* password = "19781978";

unsigned long previousMillis = 0;
const long interval = 100;

char date[20];
time_t rawtime;
struct tm* timeinfo;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const char* dayNames[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

void stopAlarm() {
  alarmActive = false;
  digitalWrite(BUZZER_PIN, LOW);
}

void startAlarm() {
  alarmActive = true;
  buzzerStartedAt = millis();
  digitalWrite(BUZZER_PIN, HIGH);
}

void updateAlarm() {
  if (alarmActive && millis() - buzzerStartedAt >= BUZZER_ON_TIME_MS) {
    stopAlarm();
  }
}

int enterOrLeave(unsigned long timeSonic, unsigned long timeIR) {
  if (timeIR > timeSonic) {
    return 1;
  }

  if (timeSonic < timeIR) {
    return -1;
  }

  return 0;
}

float readDistanceCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, SONIC_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  return (duration / 2.0f) / 29.1545f;
}

bool detectMovement(float currentCm) {
  if (currentCm < 0.0f) {
    return false;
  }

  if (previousCm == 0.0f) {
    previousCm = currentCm;
    return false;
  }

  bool moved = fabs(currentCm - previousCm) > threshold;

  if (moved) {
    previousCm = currentCm;
  }

  return moved;
}

void resetCounters() {
  peopleInCount = 0;
  peopleOutCount = 0;
  numPeople = 0;
  isIRActivated = false;
  isSonicActivated = false;
  lastEventTime = 0;
  Serial.println("Counters reset to 0");
}

void handleRemoteCommand() {
  if (IrReceiver.decodedIRData.protocol != REMOTE_PROTOCOL ||
      IrReceiver.decodedIRData.address != REMOTE_ADDRESS) {
    Serial.println("IR signal received, but protocol/address did not match.");
    return;
  }

  Serial.print("IR command received: 0x");
  Serial.println(IrReceiver.decodedIRData.command, HEX);

  if (IrReceiver.decodedIRData.command == POWER_BUTTON_COMMAND) {
    power = !power;

    if (!power) {
      stopAlarm();
    }

    Serial.print("Power toggled: ");
    Serial.println(power ? "ON" : "OFF");
  } else if (IrReceiver.decodedIRData.command == RESET_BUTTON_COMMAND) {
    resetCounters();
  } else {
    Serial.println("Button received, but it is not one of the two mapped buttons.");
  }
}

void processRemote() {
  if (!IrReceiver.decode()) {
    return;
  }

  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
    Serial.println("Overflow detected. Move farther from the receiver or increase RAW_BUFFER_LENGTH.");
  } else if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    handleRemoteCommand();
  }

  IrReceiver.resume();
}

void updateEntrySensor() {
  bool currentIRStateHigh = digitalRead(IRsensor) == HIGH;

  if (currentIRStateHigh && !lastIRStateHigh) {
    IR_ActivationTime = millis();
    isIRActivated = true;
  }

  lastIRStateHigh = currentIRStateHigh;
}

void updateSonicSensor(float currentCm) {
  if (detectMovement(currentCm)) {
    Ultra_ActivationTime = millis();
    isSonicActivated = true;
  }
}

void updateCountsFromSensors() {
  if (!isIRActivated || !isSonicActivated) {
    return;
  }

  unsigned long now = millis();
  if (now - lastEventTime <= EVENT_COOLDOWN_MS) {
    isIRActivated = false;
    isSonicActivated = false;
    return;
  }

  int direction = enterOrLeave(Ultra_ActivationTime, IR_ActivationTime);

  if (direction == 1) {
    peopleInCount++;
    numPeople++;
    Serial.println("Person entered the room.");
  } else if (direction == -1) {
    peopleOutCount++;
    if (numPeople > 0) {
      numPeople--;
    }

    Serial.println("Person left the room.");
  } else {
    Serial.println("Direction could not be determined.");
  }

  if (power && direction != 0) {
    startAlarm();
  }

  lastEventTime = now;
  isIRActivated = false;
  isSonicActivated = false;
}

void printStatus(float currentCm) {
  if (millis() - lastStatusPrintAt < STATUS_PRINT_INTERVAL_MS) {
    return;
  }

  Serial.print("power = ");
  Serial.print(power ? "ON" : "OFF");
  Serial.print(" | alarm = ");
  Serial.print(alarmActive ? "ON" : "OFF");
  Serial.print(" | peopleInCount = ");
  Serial.print(peopleInCount);
  Serial.print(" | peopleOutCount = ");
  Serial.print(peopleOutCount);
  Serial.print(" | numPeople = ");
  Serial.print(numPeople);
  Serial.print(" | distanceCm = ");

  if (currentCm < 0.0f) {
    Serial.println("no echo");
  } else {
    Serial.println(currentCm);
  }

  lastStatusPrintAt = millis();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IRsensor, INPUT);

  Serial.println();
  Serial.println("Combined security sketch started");
  Serial.println("Serial Monitor should be set to 9600 baud.");

  //monitor setup

  Serial.begin(9600);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  configTime(-28800, 3600, "pool.ntp.org");

  Serial.print("Waiting for NTP time sync");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println(" Synced!");
}

void loop() {
  processRemote();

  float currentCm = readDistanceCm();
  updateEntrySensor();
  updateSonicSensor(currentCm);
  updateCountsFromSensors();
  updateAlarm();
  printStatus(currentCm);

   unsigned long currentMillis = millis();
  // monitor code
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    int hours   = timeinfo->tm_hour;
    int minutes = timeinfo->tm_min;
    int seconds = timeinfo->tm_sec;

    // Determine AM/PM and convert to 12-hour format
    const char* ampm = (hours >= 12) ? "PM" : "AM";
    hours = hours % 12;
    if (hours == 0) hours = 12;

    // Display time (large)
    sprintf(date, "%02d:%02d:%02d", hours, minutes, seconds);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(date);

    // Display AM/PM (small, top right)
    display.setTextSize(1);
    display.setCursor(104, 0);
    display.print(ampm);

    // Display day and date
    int dayOfWeek = timeinfo->tm_wday;
    int day       = timeinfo->tm_mday;
    int month     = timeinfo->tm_mon + 1;

    char actualDate[20];
    sprintf(actualDate, "%s %02d/%02d", dayNames[dayOfWeek], month, day);
    display.setCursor(0, 18);
    display.print(actualDate);


    display.setCursor(0,32);
    display.print(numPeople);

    display.setCursor(0,44);

    char AlarmOnOrOff[30];
    sprintf(AlarmOnOrOff, "Alarm is %s", power ? "ON" : "OFF");
    display.print(AlarmOnOrOff);    

    display.display();

    
  }


}
