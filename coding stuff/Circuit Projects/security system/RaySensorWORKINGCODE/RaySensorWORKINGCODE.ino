#include <Arduino.h>
#include <IRremote.hpp>

constexpr uint8_t IR_RECEIVE_PIN = 14;
constexpr uint8_t BUZZER_PIN = 25;
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr decode_type_t REMOTE_PROTOCOL = NEC;
constexpr uint16_t REMOTE_ADDRESS = 0x0;
constexpr uint16_t POWER_BUTTON_COMMAND = 0x45;
constexpr uint16_t RESET_BUTTON_COMMAND = 0x44;

bool power = false;
int peopleInCount = 0;
int peopleOutCount = 0;

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
    Serial.print("Power toggled: ");
    Serial.println(power ? "ON" : "OFF");
  } else if (IrReceiver.decodedIRData.command == RESET_BUTTON_COMMAND) {
    peopleInCount = 0;
    peopleOutCount = 0;
    Serial.println("Room counters reset to 0");
  } else {
    Serial.println("Button received, but it is not one of the two mapped buttons.");
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println();
  Serial.println("ESP32 IR receiver started");
  Serial.println("Serial Monitor should be set to 115200 baud.");
}

void loop() {
  if (!IrReceiver.decode()) {
    digitalWrite(BUZZER_PIN, power ? HIGH : LOW);
    return;
  }

  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
    Serial.println("Overflow detected. Move farther from the receiver or increase RAW_BUFFER_LENGTH.");
  } else if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    handleRemoteCommand();
  }

  digitalWrite(BUZZER_PIN, power ? HIGH : LOW);
  IrReceiver.resume();
}
