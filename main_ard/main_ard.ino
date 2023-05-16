/* Pitch Controls (Up and Down)
   Two buttons which control the position of the elevator fins
   Controlled with one stepper motor for both fins
   One button pushes the fins down which will cause the sub to pitch downward
   One button pushes the fins up which will cause the sub to pitch upwards
*/
/* Yaw Controls (Left and Right)
   Two buttons which control the position of the Rudder fins
   Controlled with one stepper motor for both fins
   One button pushes the fins Left which will cause the sub to yaw leftward
   One button pushes the fins Right which will cause the sub to yaw rightward
*/
/* Autonomous controls button
   One button that will switch the sub into autonomous mode and the sub will make course adjustments using the gyro scope
   The switch turns the mode on and off
*/
// Gyro & 2 Pressure sensors & 2 rpm sensors & 2 motor controllers

// Define pins
#include "I2Cdev.h"

#define pitchDirPin 2
#define pitchStepPin 3
#define yawDirPin 4
#define yawStepPin 5
#define pitchUp 6
#define pitchDown 7
#define yawLeft 8
#define yawRight 9


#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 13  // use pin 13 on Arduino Uno & most boards
#define LED_PIN 10 // (Arduino is 10)
bool blinkState = false;

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

// Variables for max clockwise and counterwise step counts
#define degreesInRotation 360
const double gearRatio = 10; // Represents a 10:1 gear ratio, used for outputting degree of control surfaces
const double degree = 200;
double degreeRatio = degree/360;
const double stepCountPerRotation = 1600;
double maxClockwiseStepCount = degreeRatio * stepCountPerRotation;
double maxCounterClockwiseStepCount = degreeRatio * -stepCountPerRotation;

// Light control vars
unsigned long prevLightMicros = 0;
const long lightOffTime = 750000;
const long lightOnTime = 250000;
bool lightOn = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

const int dataINA_RPM = 11; //RPM sensor 1
const int dataINC_RPM = 12; //RPM sensor 2
const int dataINB_PS = A1; //Pressure sensor
const int dataIND_PS = A0; //Pressure sensor
const int buttonPin = A2;

unsigned long prevmillis; // To store time
unsigned long duration; // To store time difference
unsigned long refresh; // To store time for refresh of reading

int rpmA; // RPM from sensor A value
int rpmB; // RPM from sensor 2 value
int avg_rpm; // average rpm reading
int time;
bool doLoop = false;

bool state = false;
int startTime = 0;
int endTime = 0;
int totalTime = 0;

boolean currentstateA; // Current state of RPMA input scan
boolean prevstateA; // State of RPMA sensor in previous scan
boolean currentstateB; // Current state of RPMB input scan
boolean prevstateB; // State of RPMB sensor inBprevious scan

void timeSinceStart() {
  time = millis() / 1000;
}


/********** PRESSURE SENSORS ******************************************************/
void insideSensorValue() {
  // These values give us depth in inches (thats what the PS are calibrated to)
  int insideSensorVal = analogRead(dataINB_PS); // shorter PS, corresponds to depth
  Serial.print(insideSensorVal);
}

void outsideSensorValue(){
  // These values give us depth in inches (thats what the PS are calibrated to)
  int outsideSensorVal = analogRead(dataIND_PS); // Longer PS, used for calculating velocity
  Serial.print(outsideSensorVal);
}


/********** RPM/HALL EFFECT SENSORS ***********************************************/
void rpm_value()
{
  // RPMA Measurement
  currentstateA = digitalRead(dataINA_RPM); // Read RPMA sensor state
  if ( prevstateA != currentstateA) // If there is change in input
  {
    if ( currentstateA == HIGH ) // If input only changes from LOW to HIGH
    {
      duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
      rpmA = (60000000 / duration); // rpm = (1/ time millis)*1000*1000*60;
      prevmillis = micros(); // store time for nect revolution calculation
    }
    else
    {
      rpmA = 0;
    }
  }

  prevstateA = currentstateA; // store this scan (prev scan) data for next scan

  // RPMB Measurement
  currentstateB = digitalRead(dataINC_RPM); // Read RPMB sensor state
  if ( prevstateB != currentstateB) // If there is change in input
  {
    if ( currentstateB == HIGH ) // If input only changes from LOW to HIGH
    {
      duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
      rpmB = (60000000 / duration); // rpm = (1/ time millis)*1000*1000*60;
      prevmillis = micros(); // store time for next revolution calculation
    }
    else
    {
      rpmB = 0;
    }
  }
  prevstateB = currentstateB; // store this scan (prev scan) data for next scan

  // Calculating average rpm
  avg_rpm = (rpmA + rpmB) / 2;
  Serial.print(avg_rpm);
}


/********** GYRO SENSOR ***********************************************/
void gyro() {
  // if programming failed, don't try to do anything
  if (!dmpReady) {
    Serial.print("!,!");
    return;
  }
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("pry\t");
    //Serial.print(ypr[2] * 180 / M_PI); //pitch
    //Serial.print(",");
    Serial.print(ypr[1] * 180/M_PI); //roll
    Serial.print(",");
    Serial.print(ypr[0] * 180 / M_PI); //yaw

#endif

    // blink LED to indicate activity
    //blinkState = !blinkState;
    //digitalWrite(LED_PIN, blinkState);
  }
  else {
    Serial.print("!,!");
  }
}



/********** MOTORS***********************************************/
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

void printDirectionDegrees() {
  double pitchDegree = (pitchStepCount / stepCountPerRotation) * degreesInRotation;
  double yawDegree = (yawStepCount / stepCountPerRotation) * degreesInRotation;

  Serial.print(pitchDegree);
  Serial.print(",");
  Serial.println(yawDegree);
}

/**************** LIGHTS ***********************************************/
void runLights() {
  // Lights off for 750000 ms or .75 secs
  if ((currMicros - prevLightMicros) >= lightOffTime && !lightOn) {
    prevLightMicros = currMicros;
    digitalWrite(LED_PIN, HIGH);
    lightOn = true;
  }
  // Lights on for 250000 ms or .25 secs
  else if ((currMicros - prevLightMicros) >= lightOnTime && lightOn) {
    prevLightMicros = currMicros;
    digitalWrite(LED_PIN, LOW);
    lightOn = false;
  }
}


void setup() {
  // Set up for the debugging serial monitor
  Serial.begin(115200); //Start serial communication for debug statements

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // set up the buttons for the motors
  pinMode(pitchDirPin, OUTPUT);
  pinMode(pitchStepPin, OUTPUT);
  pinMode(yawDirPin, OUTPUT);
  pinMode(yawStepPin, OUTPUT);

  pinMode(yawLeft, INPUT);
  pinMode(yawRight, INPUT);
  pinMode(pitchUp, INPUT);
  pinMode(pitchDown, INPUT);

  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  // set up for the pressure sensor readings
  pinMode(dataINB_PS, INPUT);
  pinMode(dataIND_PS, INPUT);

  // set up for the RPM sensor readings
  pinMode(dataINA_RPM, INPUT);
  pinMode(dataINC_RPM, INPUT);
  prevmillis = 0;
  prevstateA = LOW;

  // STUFF FROM THE MPU6050 Lib for calculating the pitch/roll/yaw
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  while (!Serial); 

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  // END OF STUFF FROM THE MPU6050 Lib for calculating the pitch/roll/yaw

}


void loop() {
  // Get micros for motors and lights
  currMicros = micros();

  // Motor Code
  // Update motor state based on button presses and move motors one step
  updateMotorState();
  moveMotors();

  // Lights
  runLights();

  // Sensor Code
  insideSensorValue();
  Serial.print(",");
  outsideSensorValue();
  Serial.print(",");
  rpm_value();
  Serial.print(",");
  gyro();

  // Print control surface degree values
  Serial.print(",");
  printDirectionDegrees();
}
