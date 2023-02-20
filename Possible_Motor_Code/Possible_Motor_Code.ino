// Pins for the stepper motors
const int pitchDirPin = 2;
const int pitchStepPin = 3;
const int yawDirPin = 4;
const int yawStepPin = 5;

// Pins for the buttons
const int pitchUp = 6;
const int pitchDown = 7;
const int yawLeft = 8;
const int yawRight = 9;

// Delay between steps in microseconds and counter for micros
const unsigned long microsBetweenSteps = 1000;
unsigned long prevMicros = 0;

// Variable to keep track of micros count for motors and lights
unsigned long currMicros = 0;

// Variables to keep track of motor state
bool pitchMoving = false;
bool yawMoving = false;
bool pitchClockwise = false;
bool yawClockwise = false;

// Counter variables for motor steps
int pitchStepCount = 0;
int yawStepCount = 0;
int maxClockwiseStepCount = 800;
int maxCounterClockwiseStepCount = -800;

// Read the state of the buttons and update motor state accordingly
void updateMotorState() {
  bool buttonLeftPushed = digitalRead(yawLeft) == HIGH;
  bool buttonRightPushed = digitalRead(yawRight) == HIGH;
  bool buttonUpPushed = digitalRead(pitchUp) == HIGH;
  bool buttonDownPushed = digitalRead(pitchDown) == HIGH;

  // Determine which motor(s) should be moving and in which direction
  if (buttonLeftPushed && !buttonRightPushed) {
    yawMoving = true;
    yawClockwise = true;
  }
  else if (buttonRightPushed && !buttonLeftPushed) {
    yawMoving = true;
    yawClockwise = false;
  }
  else {
    yawMoving = false;
  }

  if (buttonUpPushed && !buttonDownPushed) {
    pitchMoving = true;
    pitchClockwise = true;
  }
  else if (buttonDownPushed && !buttonUpPushed) {
    pitchMoving = true;
    pitchClockwise = false;
  }
  else {
    pitchMoving = false;
  }
}

// Move the motors one step in the appropriate direction(s)
void moveMotors() {
  if (currMicros - prevMicros >= microsBetweenSteps) {
    prevMicros = currMicros;
    
    if (pitchMoving) {
      if (pitchClockwise && pitchStepCount < maxClockwiseStepCount) {
        // Set the direction pin to for clockwise
        digitalWrite(pitchDirPin, HIGH);

        // Toggle the step pin to move one step
        digitalWrite(pitchStepPin, HIGH);
        digitalWrite(pitchStepPin, LOW);
        pitchStepCount++;
      }
      else if (!pitchClockwise && pitchStepCount > maxCounterClockwiseStepCount) {
        // Set the direction pin to for counter clockwise
        digitalWrite(pitchDirPin, LOW);

        // Toggle the step pin to move one step
        digitalWrite(pitchStepPin, HIGH);
        digitalWrite(pitchStepPin, LOW);
        pitchStepCount--;
      }
    }

    if (yawMoving) {
      if (yawClockwise && yawStepCount < maxClockwiseStepCount) {
        // Set the direction pin to for clockwise
        digitalWrite(yawDirPin, HIGH);

        // Toggle the step pin to move one step
        digitalWrite(yawStepPin, HIGH);
        digitalWrite(yawStepPin, LOW);
        yawStepCount++;
      }
      else if (!yawClockwise && yawStepCount > maxCounterClockwiseStepCount) {
        // Set the direction pin to for counter clockwise
        digitalWrite(yawDirPin, LOW);

        // Toggle the step pin to move one step
        digitalWrite(yawStepPin, HIGH);
        digitalWrite(yawStepPin, LOW);
        yawStepCount--;
      }
    }
  }
}

void setup() {
  // Set pin modes
  pinMode(pitchDirPin, OUTPUT);
  pinMode(pitchStepPin, OUTPUT);
  pinMode(yawDirPin, OUTPUT);
  pinMode(yawStepPin, OUTPUT);
  pinMode(yawLeft, INPUT_PULLUP);
  pinMode(yawRight, INPUT_PULLUP);
  pinMode(pitchUp, INPUT_PULLUP);
  pinMode(pitchDown, INPUT_PULLUP);
}

void loop() {
  currMicros = micros();
  // Update motor state based on button presses and move motors one step
  updateMotorState();
  moveMotors();
}
