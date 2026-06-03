int trigPin = 18;    // Trigger
int echoPin = 19;    // Echo

//IR sensor
int IRsensor = 23;
//default state
int state = LOW;
//sensor status
int val = 0; 

long duration, cm, inches;
const float threshold = 10.0; // how many cm counts as "movement"
float previousCm = 0; //compare the distance before vs now

//exact time when the IR triggers
long IR_ActivationTime;
//exact time when ultrasound triggers
long Ultra_ActivationTime;

//number of people in a room
int numPeople = 0;

long currentMil = 0;

// Cooldown to prevent re-triggering while someone lingers
const long EVENT_COOLDOWN_MS = 2000;
long lastEventTime = 0;

bool isIRActivated = false;
bool isSonicActivated = false;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IRsensor, INPUT);
}

// Returns 1 if someone entered, -1 if someone left, 0 otherwise
int enterOrLeave(long timeSonic, long timeIR) {
  if (timeIR < timeSonic) return 1;
  if (timeSonic < timeIR) return -1;
  return 0;
}

// Detects movement using the ultrasonic sensor
bool detectMovement() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 26000);
  float currentCM = (duration / 2.0) / 29.1545;

  bool moved = abs(currentCM - previousCm) > threshold;

  // Only update baseline on real movement so noise doesn't drift previousCm
  if (moved) {
    previousCm = currentCM;
  }

  return moved;
}

void loop() {
  val = digitalRead(IRsensor);

  if (val == HIGH) {
    IR_ActivationTime = millis();
    isIRActivated = true;
  }

  if (detectMovement()) {
    Ultra_ActivationTime = millis();
    isSonicActivated = true;
  }

  if (isSonicActivated && isIRActivated) {
    long now = millis();

    // Only count an event if we're outside the cooldown window
    if (now - lastEventTime > EVENT_COOLDOWN_MS) {
      numPeople += enterOrLeave(Ultra_ActivationTime, IR_ActivationTime);
      lastEventTime = now;
    }

    // Always reset flags to prevent stale re-triggers
    isIRActivated = false;
    isSonicActivated = false;
  }

  if (millis() > currentMil) {
    Serial.println(numPeople);
    currentMil = millis() + 1000;
  }
}