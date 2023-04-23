/* Pitch Controls (Up and Down)
 * Two buttons which control the position of the elevator fins
 * Controlled with one stepper motor for both fins
 * One button pushes the fins down which will cause the sub to pitch downward
 * One button pushes the fins up which will cause the sub to pitch upwards
 */
/* Yaw Controls (Left and Right)
 * Two buttons which control the position of the Rudder fins
 * Controlled with one stepper motor for both fins
 * One button pushes the fins Left which will cause the sub to yaw leftward
 * One button pushes the fins Right which will cause the sub to yaw rightward
 */
/* Autonomous controls button
 * One button that will switch the sub into autonomous mode and the sub will make course adjustments using the gyro scope
 * The switch turns the mode on and off
 */
// Gyro
// 2 stepper motors (Step Pulse Control and Direction Control)
// 

// Defin pins
 
#define pitchDirPin 2
#define pitchStepPin 3
#define yawDirPin 4
#define yawStepPin 5
//#define pitchUp 6
//#define pitchDown 7
//#define yawLeft 8
//#define yawRight 9
#define stepsPerRevolution 1600
 
// Variables

int sec = 1; // Amount of time for motor to run
int pd = 250; // Pulse Delay period
boolean setPitchDir = LOW; // Set Pitch Direction
boolean setYawDir = LOW; // Set Pitch Direction

// Switches direction of up or down
void switchPitch (){
 
  setPitchDir = !setPitchDir;
  
}

// Switches direction left or right
void switchYaw (){
 
  setYawDir = !setYawDir;
  
}

// Runs pitch motor for 5 seconds
// Temporary
void runPitch(){
  
  //for (int i = (sec * 1000000)/(2 * pd); i >= 0; i--) {
  for (int i = stepsPerRevolution; i >= 0; i--) {
    digitalWrite(pitchDirPin, setPitchDir);
    digitalWrite(pitchStepPin, HIGH);
    //delayMicroseconds(pd);
    digitalWrite(pitchStepPin, LOW);
    delayMicroseconds(pd);
  }
  
}

// Runs yaw motor for 5 seconds
// Temporary
void runYaw() {
  
  //for (int i = (sec * 1000000)/(2 * pd); i >= 0; i--) {
  for (int i = stepsPerRevolution; i >= 0; i--) {
    digitalWrite(yawDirPin, setYawDir);
    digitalWrite(yawStepPin, HIGH);
    //delayMicroseconds(pd);
    digitalWrite(yawStepPin, LOW);
    delayMicroseconds(pd);
  }
  
}

void setup() {
  Serial.begin(9600);
  
  pinMode (pitchDirPin, OUTPUT);
  pinMode (pitchStepPin, OUTPUT);
  pinMode (yawDirPin, OUTPUT);
  pinMode (yawStepPin, OUTPUT);
//  pinMode (pitchUp, INPUT);
//  pinMode (pitchDown, INPUT);
//  pinMode (yawLeft, INPUT);
//  pinMode (yawRight, INPUT);
}

void loop() {

  if (Serial.available()) {
      char curr = Serial.read();
      while(Serial.available()) {
        Serial.read();
      }
      
      if (curr == 'a') {
        switchPitch();
      }
      else if (curr == 'b') {
        switchYaw();
      }
      else if (curr == 'p') {
        runPitch();
      }
      else if (curr == 'y') {
        runYaw();
      }
    }

}
