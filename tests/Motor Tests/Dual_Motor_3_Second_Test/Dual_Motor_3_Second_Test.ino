// Pin definitions for motor control
const int pitchStepPin = 2;
const int pitchDirPin = 3;
const int yawStepPin = 4;
const int yawDirPin = 5;

// Define motor directions
const int CW = HIGH;
const int CCW = LOW;

// Microsecond delay between motor steps
const int microsBetweenSteps = 1000;

// Run each motor for 3 seconds in each direction
void motorTest() {
  // Set pitch motor direction
  digitalWrite(pitchDirPin, CW);
  digitalWrite(yawDirPin, CW);

  // Run both motors for 3 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    digitalWrite(pitchStepPin, HIGH);
    digitalWrite(yawStepPin, HIGH);
    delayMicroseconds(microsBetweenSteps);
    digitalWrite(pitchStepPin, LOW);
    digitalWrite(yawStepPin, LOW);
    delayMicroseconds(microsBetweenSteps);
  }

  // Set pitch motor direction
  digitalWrite(pitchDirPin, CCW);
  digitalWrite(yawDirPin, CCW);

  // Run both motors for 3 seconds
  startTime = millis();
  while (millis() - startTime < 3000) {
    digitalWrite(pitchStepPin, HIGH);
    digitalWrite(yawStepPin, HIGH);
    delayMicroseconds(microsBetweenSteps);
    digitalWrite(pitchStepPin, LOW);
    digitalWrite(yawStepPin, LOW);
    delayMicroseconds(microsBetweenSteps);
  }
}

void setup() {
  // Set motor control pins as outputs
  pinMode(pitchStepPin, OUTPUT);
  pinMode(pitchDirPin, OUTPUT);
  pinMode(yawStepPin, OUTPUT);
  pinMode(yawDirPin, OUTPUT);
}

void loop() {
  motorTest();
  delay(5000); // Pause for 5 second between tests
}
