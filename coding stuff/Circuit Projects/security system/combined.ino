#include <Arduino.h>
#include <IRremote.hpp>

constexpr uint8_t IR_RECEIVE_PIN = 14;
constexpr uint8_t BUZZER_PIN = 25;
constexpr uint8_t TRIG_PIN = 12;
constexpr uint8_t ECHO_PIN = 14;

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr decode_type_t REMOTE_PROTOCOL = NEC;
constexpr uint16_t REMOTE_ADDRESS = 0x0;
constexpr uint16_t POWER_BUTTON_COMMAND = 0x45;
constexpr uint16_t RESET_BUTTON_COMMAND = 0x44;

constexpr unsigned long BUZZER_ON_TIME_MS = 5000;
constexpr unsigned long STATUS_PRINT_INTERVAL_MS = 1000;
constexpr unsigned long ULTRASONIC_TIMEOUT_US = 30000;

// Tune these thresholds to match your doorway distance.
constexpr float PERSON_DETECTED_CM = 60.0f;
constexpr float PERSON_CLEARED_CM = 90.0f;

bool power = false;
bool personPresent = false;
bool buzzerActive = false;

int peopleInCount = 0;
int peopleOutCount = 0;

unsigned long buzzerStartedAt = 0;
unsigned long lastStatusPrintAt = 0;

void stopBuzzer() {
  buzzerActive = false;
  digitalWrite(BUZZER_PIN, LOW);
}

void startBuzzer() {
  buzzerActive = true;
  buzzerStartedAt = millis();
  digitalWrite(BUZZER_PIN, HIGH);
}

void updateBuzzer() {
  if (buzzerActive && millis() - buzzerStartedAt >= BUZZER_ON_TIME_MS) {
    stopBuzzer();
  }
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  return (duration / 2.0f) / 29.1f;
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
      stopBuzzer();
    }

    Serial.print("Power toggled: ");
    Serial.println(power ? "ON" : "OFF");
  } else if (IrReceiver.decodedIRData.command == RESET_BUTTON_COMMAND) {
    peopleInCount = 0;
    peopleOutCount = 0;
    personPresent = false;
    Serial.println("Room counters reset to 0");
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

void updatePeopleCounts(float distanceCm) {
  if (distanceCm < 0.0f) {
    return;
  }

  if (!personPresent && distanceCm <= PERSON_DETECTED_CM) {
    personPresent = true;
    peopleInCount++;
    Serial.print("Person detected entering. In count: ");
    Serial.println(peopleInCount);

    if (power) {
      startBuzzer();
    }
  } else if (personPresent && distanceCm >= PERSON_CLEARED_CM) {
    personPresent = false;
    peopleOutCount++;
    Serial.print("Person detected leaving. Out count: ");
    Serial.println(peopleOutCount);

    if (power) {
      startBuzzer();
    }
  }
}

void printStatus(float distanceCm) {
  if (millis() - lastStatusPrintAt < STATUS_PRINT_INTERVAL_MS) {
    return;
  }

  Serial.print("power = ");
  Serial.print(power ? "ON" : "OFF");
  Serial.print(" | peopleInCount = ");
  Serial.print(peopleInCount);
  Serial.print(" | peopleOutCount = ");
  Serial.print(peopleOutCount);
  Serial.print(" | distanceCm = ");

  if (distanceCm < 0.0f) {
    Serial.println("no echo");
  } else {
    Serial.println(distanceCm);
  }

  lastStatusPrintAt = millis();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println();
  Serial.println("Combined security sketch started");
  Serial.println("Serial Monitor should be set to 115200 baud.");
}

void loop() {
  processRemote();

  float distanceCm = readDistanceCm();
  updatePeopleCounts(distanceCm);
  updateBuzzer();
  printStatus(distanceCm);

  delay(100);
}
