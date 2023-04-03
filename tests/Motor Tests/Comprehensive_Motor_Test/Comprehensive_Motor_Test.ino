// Pin definitions for motor control
#define pitchStepPin 2
#define pitchDirPin 3
#define yawStepPin 4
#define yawDirPin 5

unsigned long startTime = 0;
#define microsBetweenSteps 1000

void setup() {
  // Set motor control pins as outputs
  pinMode(pitchStepPin, OUTPUT);
  pinMode(pitchDirPin, OUTPUT);
  pinMode(yawStepPin, OUTPUT);
  pinMode(yawDirPin, OUTPUT);
}

void loop() {
  pitchIndividualTest();
  yawIndividualTest();
  combinedSameDirectionTest();
  combinedOppDirectionTest();
  delay(5000); // Delay 5 seconds in between test runs
}

void pitchIndividualTest() {
  digitalWrite(pitchDirPin, HIGH);
  // Run pitch motor clockwise for 3 seconds
  startTime = millis();
  while (millis() - startTime < 3000) {
    digitalWrite(pitchStepPin, HIGH);
    delayMicroseconds(microsBetweenSteps);
    digitalWrite(pitchStepPin, LOW);
    delayMicroseconds(microsBetweenSteps);
  }

  digitalWrite(pitchDirPin, LOW); 
  // Run pitch motor counter clockwise for 3 seconds
  startTime = millis();
  while (millis() - startTime < 3000) {
    digitalWrite(pitchStepPin, HIGH);
    delayMicroseconds(microsBetweenSteps);
    digitalWrite(pitchStepPin, LOW);
    delayMicroseconds(microsBetweenSteps);
  }
}

void yawIndividualTest() {
  digitalWrite(yawDirPin, HIGH);
  // Run yaw motor clockwise for 3 seconds
  startTime = millis();
  while (millis() - startTime < 3000) {
    digitalWrite(yawStepPin, HIGH);
    delayMicroseconds(microsBetweenSteps);
    digitalWrite(yawStepPin, LOW);
    delayMicroseconds(microsBetweenSteps);
  }

  digitalWrite(yawDirPin, LOW);
  // Run yaw motor counter clockwise for 3 seconds
  startTime = millis();
  while (millis() - startTime < 3000) {
    digitalWrite(yawStepPin, HIGH);
    delayMicroseconds(microsBetweenSteps);
    digitalWrite(yawStepPin, LOW);
    delayMicroseconds(microsBetweenSteps);
  }
}

void combinedSameDirectionTest() {
  digitalWrite(pitchDirPin, HIGH);
  digitalWrite(yawDirPin, HIGH);
  // Run both motors clockwise for 3 seconds
  startTime = millis();
  while (millis() - startTime < 3000) {
    digitalWrite(pitchStepPin, HIGH);
    digitalWrite(yawStepPin, HIGH);
    delayMicroseconds(microsBetweenSteps);
    digitalWrite(pitchStepPin, LOW);
    digitalWrite(yawStepPin, LOW);
    delayMicroseconds(microsBetweenSteps);
  }

  digitalWrite(pitchDirPin, LOW);
  digitalWrite(yawDirPin, LOW);
  // Run both motors counter clockwise for 3 seconds
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

void combinedOppDirectionTest() {
  digitalWrite(pitchDirPin, HIGH);
  digitalWrite(yawDirPin, LOW);
  // Run pitch motor clockwise and yaw motor counter clockwise for 3 seconds
  startTime = millis();
  while (millis() - startTime < 3000) {
    digitalWrite(pitchStepPin, HIGH);
    digitalWrite(yawStepPin, HIGH);
    delayMicroseconds(microsBetweenSteps);
    digitalWrite(pitchStepPin, LOW);
    digitalWrite(yawStepPin, LOW);
    delayMicroseconds(microsBetweenSteps);
  }

  digitalWrite(pitchDirPin, LOW);
  digitalWrite(yawDirPin, HIGH);
  // Run pitch motor counter clockwise and yaw motor clockwise for 3 seconds
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
