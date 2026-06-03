#include <Arduino.h>
#include <IRremote.hpp>
#include <math.h>

constexpr uint8_t IR_RECEIVE_PIN = 14;
constexpr uint8_t BUZZER_PIN = 25;
constexpr uint8_t ULTRASONIC_TRIG_PIN = 18;
constexpr uint8_t ULTRASONIC_ECHO_PIN = 19;
constexpr uint8_t ENTRY_IR_SENSOR_PIN = 23;

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr decode_type_t REMOTE_PROTOCOL = NEC;
constexpr uint16_t REMOTE_ADDRESS = 0x0;
constexpr uint16_t POWER_BUTTON_COMMAND = 0x45;
constexpr uint16_t RESET_BUTTON_COMMAND = 0x44;

constexpr float MOVEMENT_THRESHOLD_CM = 10.0f;
constexpr unsigned long ULTRASONIC_TIMEOUT_US = 26000;
constexpr unsigned long SENSOR_PAIR_WINDOW_MS = 1500;
constexpr unsigned long EVENT_COOLDOWN_MS = 2000;
constexpr unsigned long BUZZER_ON_TIME_MS = 5000;

bool powerOn = false;
bool entryIrTriggered = false;
bool ultrasonicTriggered = false;
bool lastEntryIrStateHigh = false;
bool hasBaselineDistance = false;

unsigned long entryIrTriggeredAt = 0;
unsigned long ultrasonicTriggeredAt = 0;
unsigned long lastEventTime = 0;
unsigned long buzzerOffAt = 0;

float previousDistanceCm = 0.0f;

int peopleInCount = 0;
int peopleOutCount = 0;
int roomCount = 0;

bool hasElapsed(unsigned long startTime, unsigned long durationMs) {
  return millis() - startTime >= durationMs;
}

void stopBuzzer() {
  digitalWrite(BUZZER_PIN, LOW);
  buzzerOffAt = 0;
}

void startBuzzerTimer() {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerOffAt = millis() + BUZZER_ON_TIME_MS;
}

void updateBuzzer() {
  if (buzzerOffAt != 0 && static_cast<long>(millis() - buzzerOffAt) >= 0) {
    stopBuzzer();
  }
}

void resetDetectionFlags() {
  entryIrTriggered = false;
  ultrasonicTriggered = false;
  entryIrTriggeredAt = 0;
  ultrasonicTriggeredAt = 0;
}

void resetCounters() {
  peopleInCount = 0;
  peopleOutCount = 0;
  roomCount = 0;
  resetDetectionFlags();
  Serial.println("Room counters reset to 0");
}

float readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  return (duration / 2.0f) / 29.1545f;
}

bool detectUltrasonicMovement() {
  float currentDistanceCm = readDistanceCm();
  if (currentDistanceCm < 0.0f) {
    return false;
  }

  if (!hasBaselineDistance) {
    previousDistanceCm = currentDistanceCm;
    hasBaselineDistance = true;
    return false;
  }

  float distanceDelta = fabs(currentDistanceCm - previousDistanceCm);
  previousDistanceCm = currentDistanceCm;
  return distanceDelta >= MOVEMENT_THRESHOLD_CM;
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
    powerOn = !powerOn;
    if (!powerOn) {
      stopBuzzer();
    }

    Serial.print("Power toggled: ");
    Serial.println(powerOn ? "ON" : "OFF");
  } else if (IrReceiver.decodedIRData.command == RESET_BUTTON_COMMAND) {
    resetCounters();
  } else {
    Serial.println("Button received, but it is not one of the two mapped buttons.");
  }
}

void processIrRemote() {
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

void captureEntryIrTrigger() {
  bool entryIrStateHigh = digitalRead(ENTRY_IR_SENSOR_PIN) == HIGH;

  if (entryIrStateHigh && !lastEntryIrStateHigh && !entryIrTriggered) {
    entryIrTriggered = true;
    entryIrTriggeredAt = millis();
    Serial.println("Entry IR sensor triggered.");
  }

  lastEntryIrStateHigh = entryIrStateHigh;
}

void captureUltrasonicTrigger() {
  if (detectUltrasonicMovement() && !ultrasonicTriggered) {
    ultrasonicTriggered = true;
    ultrasonicTriggeredAt = millis();
    Serial.println("Ultrasonic movement detected.");
  }
}

void clearStaleSensorTriggers() {
  if (entryIrTriggered && hasElapsed(entryIrTriggeredAt, SENSOR_PAIR_WINDOW_MS)) {
    entryIrTriggered = false;
    entryIrTriggeredAt = 0;
  }

  if (ultrasonicTriggered && hasElapsed(ultrasonicTriggeredAt, SENSOR_PAIR_WINDOW_MS)) {
    ultrasonicTriggered = false;
    ultrasonicTriggeredAt = 0;
  }
}

void handleRoomEvent() {
  if (!entryIrTriggered || !ultrasonicTriggered) {
    return;
  }

  unsigned long firstTrigger = min(entryIrTriggeredAt, ultrasonicTriggeredAt);
  unsigned long secondTrigger = max(entryIrTriggeredAt, ultrasonicTriggeredAt);
  if (secondTrigger - firstTrigger > SENSOR_PAIR_WINDOW_MS) {
    resetDetectionFlags();
    return;
  }

  if (lastEventTime != 0 && !hasElapsed(lastEventTime, EVENT_COOLDOWN_MS)) {
    resetDetectionFlags();
    return;
  }

  int direction = 0;
  if (entryIrTriggeredAt < ultrasonicTriggeredAt) {
    direction = 1;
  } else if (ultrasonicTriggeredAt < entryIrTriggeredAt) {
    direction = -1;
  }

  if (direction == 1) {
    peopleInCount++;
    roomCount++;
    Serial.print("Person entered. Room count: ");
    Serial.println(roomCount);
  } else if (direction == -1) {
    peopleOutCount++;
    if (roomCount > 0) {
      roomCount--;
    }

    Serial.print("Person left. Room count: ");
    Serial.println(roomCount);
  } else {
    Serial.println("Sensors triggered at the same time. Direction unknown.");
  }

  if (powerOn && direction != 0) {
    startBuzzerTimer();
    Serial.println("Alarm buzzer on for 5 seconds.");
  }

  lastEventTime = millis();
  resetDetectionFlags();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

  pinMode(BUZZER_PIN, OUTPUT);
  stopBuzzer();

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ENTRY_IR_SENSOR_PIN, INPUT);

  Serial.println();
  Serial.println("Combined security sketch started.");
  Serial.println("Power button toggles the system. Reset button clears room counters.");
}

void loop() {
  processIrRemote();
  captureEntryIrTrigger();
  captureUltrasonicTrigger();
  handleRoomEvent();
  clearStaleSensorTriggers();
  updateBuzzer();
}
