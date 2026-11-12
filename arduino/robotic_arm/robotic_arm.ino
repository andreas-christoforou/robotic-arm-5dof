/*  5-DOF Robotic Arm with Dual Sonars (HC-SR04)

  Hardware:
   - 5 servos driven by a PCA9685 at I2C address 0x40
   - Motor index mapping:
       0 = gripper
       1 = wrist
       2 = elbow
       3 = shoulder
       4 = base
   - Grip Sonar on pins:  TRIG = 7, ECHO = 6
   - Shoulder Sonar on pins: TRIG = 8, ECHO = 9

  Behavior:
   1) 1st scanning phase (shoulder sonar) with base sweeping 0°→150°
   2) Move arm to a ready pose based on distance
   3) 2nd scanning phase (grip sonar) with base scanning around detected angle 
   4) Pickup: move down, close gripper, lift
   5) Place: rotate to place angle and drop, then return to home
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> //PWM Library

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);


//  ==Servo Constants==

// Motor indices
const int motorGrip    = 0;
const int motorWrist   = 1;
const int motorElbow   = 2;
const int motorShoulder = 3;
const int motorBase    = 4;

const int numMotors = 5;

// PCA9685 PWM parameters
const float servoFreq = 50.0;       // 50 Hz analog servos
const uint16_t servoMinUS[numMotors] = {600, 600, 600, 600, 600};   // microseconds min
const uint16_t servoMaxUS[numMotors] = {2400,2400,2400,2400,2400};  // microseconds max
const uint16_t PWMfullScale = 4096;

// Current servo angles
int currentAngle[numMotors] = {90, 90, 90, 90, 90};  // all servos at 90 degrees at start

//  ==Sonar Config==

// Grip sonar - near gripper, downward
const int gripTrigPin = 7;
const int gripEchoPin = 6;

// Shoulder sonar - near shoulder, forward
const int shoulderTrigPin = 8;
const int shoulderEchoPin = 9;

// Sonar reading config
const int sonarSamples = 5;         // readings average
const long sonarTimeoutUS = 30000; // 30 ms timeout in pulses

//  Distance & Detection Config

// First-phase detection range (cm) - shoulder sonar
const float firstDetectMinCM = 10.0;  // ignore too close readings
const float firstDetectMaxCM = 35.0;  // ignore too far readings

// Second-phase detection threshold (cm) - grip sonar
const float secondDetectCM     = 10.0; // detection cm for how far the object needs to be to be detected
const float pickApproachCM     = 5.0;  // force stop vertical descent at 5cm

// Debounce for GRIP sonar detection (second scan)
unsigned long gripDetectStart = 0;              // first observation of the object
const unsigned long gripDetectHoldMS = 250;     // time to stay visible in order to pick it up

//  ==Angle Configs==

// Base scanning range (first phase)
const int baseScanMin = 0;      // minimum degrees of 1st scan (Left)
const int baseScanMax = 150;    // maximum degrees of 1st scan (Right)
const int baseScanStep = 3;     // degrees per step 
const int baseScanStepDelay = 10; // ms between steps

// Base scanning range (second phase) around detected angle
const int secondScanArc = 30;    // +/- around detected base angle
const int secondScanStep = 2;
const int secondScanStepDelay = 50; // slower to be more precise

// Base angle for place location
const int placeBaseAngle = 180; //constant far right

// Gripper angles
const int gripOpenAngle  = 0; // fully open
const int gripCloseAngle = 180; // fully closed

//  ==Telemetry Globals (For Processing Code)== 

unsigned long lastTelemetryMs = 0;
const unsigned long telemetryIntervalMS = 100; // send to Processing every 100 ms

// Pre-defined Poses
// Order: [grip, wrist, elbow, shoulder, base]

// Home / idle pose
int poseHome[numMotors] = {
  gripOpenAngle,   
  0,                
  180,             
  140,              
  90 
};

// Ready poses for different distances

// near (10–18 cm)
int poseReadyNear[numMotors] = {
  gripOpenAngle,
  10,   
  180,  
  120,  
  90
};

// mid (18–28 cm)
int poseReadyMid[numMotors] = {
  gripOpenAngle,
  0,
  140,
  90,
  90
};

// far (28–40 cm) (does not work with our current object but wont remove)
int poseReadyFar[numMotors] = {
  gripOpenAngle,
  0,
  120,
  70,
  90
};

// Lift pose after pickup
int poseLift[numMotors] = {
  gripCloseAngle,
  0,
  140,
  110,
  90
};

// Place pose (arm over drop area)
int posePlace[numMotors] = {
  gripCloseAngle,
  0,
  150,
  100,
  placeBaseAngle
};

//  ==State Machine==

enum RobotState {
  stateFirstScan,
  stateMoveReady,
  stateSecondScan,
  statePickup,
  statePlace
};

RobotState state = stateFirstScan;

// Data from first scanning phase
bool firstDetectFound       = false;
float firstDetectDistanceCm = 0.0;
int   firstDetectBaseAngle  = 90;

// Data from second scanning phase
bool secondDetectFound      = false;
int  secondDetectBaseAngle  = 90;

//  ==Servo Functions==

uint16_t angleToTicks(int motorIndex, int angleDeg) {
  if (angleDeg < 0)   angleDeg = 0;
  if (angleDeg > 180) angleDeg = 180;

  uint16_t usMin = servoMinUS[motorIndex];
  uint16_t usMax = servoMaxUS[motorIndex];

  float span = (float)(usMax - usMin);
  float us   = usMin + (span * angleDeg / 180.0);

  // convert microseconds to PCA9685 ticks
  float ticksPerUs = (PWMfullScale * servoFreq) / 1000000.0;
  float ticks = us * ticksPerUs;
  if (ticks < 0)         ticks = 0;
  if (ticks > 4095)      ticks = 4095;

  return (uint16_t)(ticks + 0.5);
}

void setServoAngleInstant(int motorIndex, int angleDeg) {
  uint16_t ticks = angleToTicks(motorIndex, angleDeg);
  pwm.setPWM(motorIndex, 0, ticks);
  currentAngle[motorIndex] = angleDeg;
}

void sendTelemetry() {
  float dG = readSonarOnce(gripTrigPin, gripEchoPin);
  float dS = readSonarOnce(shoulderTrigPin, shoulderEchoPin);
  int   ba = currentAngle[motorBase];

  Serial.print("S1,");
  Serial.print(dG, 1);
  Serial.print(",S2,");
  Serial.print(dS, 1);
  Serial.print(",BA,");
  Serial.println(ba);
}


// Smooth move from currentAngle[motorIndex] to targetAngle
// stepDelay (ms) controls speed
void moveServoSmooth(int motorIndex, int targetAngle, int stepDelayMs) {
  if (targetAngle < 0)   targetAngle = 0;
  if (targetAngle > 180) targetAngle = 180;

  int start = currentAngle[motorIndex];
  int step  = (targetAngle > start) ? 1 : -1;

  for (int a = start; a != targetAngle; a += step) {
    setServoAngleInstant(motorIndex, a);

    // Sending live telemetry to processing code
    unsigned long now = millis();
    if (now - lastTelemetryMs >= telemetryIntervalMS) {
      lastTelemetryMs = now;
      sendTelemetry();
    }

    delay(stepDelayMs);
  }

  // ensure exact final angle
  setServoAngleInstant(motorIndex, targetAngle);
}


// Move all 5 motors to the given pose smoothly (same stepDelay for all)
void movePoseSmooth(int pose[numMotors], int stepDelayMs) {
  for (int i = 0; i < numMotors; i++) {
    moveServoSmooth(i, pose[i], stepDelayMs);
  }
}

//  Sonar reading with stabilization

// Read one distance measurement (cm) from HC-SR04
float readSonarOnce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, sonarTimeoutUS);
  if (duration == 0) {
    return 9999.0; // remove echo
  }
  float cm = duration / 58.0; // conversion
  return cm;
}

// Take multiple readings, average valid values, ignore 0 or big ones
float readSonarStable(int trigPin, int echoPin) {
  float sum = 0.0;
  int validCount = 0;

  for (int i = 0; i < sonarSamples; i++) {
    float d = readSonarOnce(trigPin, echoPin);
    if (d > 0.0 && d < 400.0) { // basic bounds
      sum += d;
      validCount++;
    }
    delay(10);
  }

  if (validCount == 0) return 9999.0;
  return sum / validCount;
}

//  ==Arm Behaviours==

// --First Scanning Phase--
// Sweep base from baseScanMin to baseScanMax using shoulder sonar
// If object in firstDetectMinCM to firstDetectMaxCM is found:
//   - store firstDetectBaseAngle, firstDetectDistanceCm
//   - set firstDetectFound = true
void scanFirstPhase() {
  firstDetectFound = false;
  Serial.println(F("== FIRST SCAN: sweeping with shoulder sonar =="));

  // Ensure arm is in a good scan pose
  movePoseSmooth(poseHome, 10);

  // Sweep base from left to right
  for (int angle = baseScanMin; angle <= baseScanMax; angle += baseScanStep) {
    moveServoSmooth(motorBase, angle, baseScanStepDelay);

    float d = readSonarStable(shoulderTrigPin, shoulderEchoPin);
    Serial.print(F("First Scan - base="));
    Serial.print(angle);
    Serial.print(F(" deg, shoulder sonar="));
    Serial.print(d);
    Serial.println(F(" cm"));

    if (d >= firstDetectMinCM && d <= firstDetectMaxCM) {
      // Object detected in valid range
      firstDetectFound       = true;
      firstDetectBaseAngle   = angle;
      firstDetectDistanceCm  = d;
      Serial.println(F(">>> FIRST SCAN: Object detected!"));
      Serial.print(F("    baseAngle=")); Serial.println(firstDetectBaseAngle);
      Serial.print(F("    distance="));  Serial.println(firstDetectDistanceCm);
      break;
    }
  }

  if (!firstDetectFound) {
    Serial.println(F("FIRST SCAN: no object found in this sweep."));
  }
}

// --Move Arm to Ready Position--
// Based on firstDetectDistanceCM choose one of the poseReady* presets.
// Rotate base to firstDetectBaseAngle, then move joints.
void moveToReadyPosition() {
  Serial.println(F("== MOVE TO READY POSITION =="));

  // If nothing found, just re-center base and stay in home
  if (!firstDetectFound) {
    Serial.println(F("No object from first scan, staying in HOME pose."));
    movePoseSmooth(poseHome, 10);
    return;
  }

  // 1) Rotate base to where we saw the object
  moveServoSmooth(motorBase, firstDetectBaseAngle, 10);

  // 2) Choose pose based on distance bands
  int* selectedPose;
  if (firstDetectDistanceCm < 18.0) {
    selectedPose = poseReadyNear;
    Serial.println(F("Distance band: NEAR"));
  } else if (firstDetectDistanceCm < 28.0) {
    selectedPose = poseReadyMid;
    Serial.println(F("Distance band: MID"));
  } else {
    selectedPose = poseReadyFar;
    Serial.println(F("Distance band: FAR"));
  }

  // 3) Move all motors (except base which is already in place) to ready pose
  int tempPose[numMotors];
  for (int i = 0; i < numMotors; i++) {
    tempPose[i] = selectedPose[i];
  }
  // preserve base angle
  tempPose[motorBase] = firstDetectBaseAngle;

  movePoseSmooth(tempPose, 10);
}

//  --Second Scanning Phase--
// With arm in ready pose and base at firstDetectBaseAngle,
// scan base left→right in a smaller arc using grip sonar.
// If grip sonar < secondDetectCM, store secondDetectBaseAngle.
void scanSecondPhase() {
  secondDetectFound = false;
  gripDetectStart   = 0;   // reset timer at start
  Serial.println(F("== SECOND SCAN: fine scan with grip sonar =="));

  if (!firstDetectFound) {
    Serial.println(F("No first detection; skipping second scan."));
    return;
  }

  int centerAngle = firstDetectBaseAngle;
  int leftLimit   = centerAngle - secondScanArc;
  int rightLimit  = centerAngle + secondScanArc;

  if (leftLimit < baseScanMin)  leftLimit  = baseScanMin;
  if (rightLimit > baseScanMax) rightLimit = baseScanMax;

  // Sweep from left to right around the center
  for (int angle = leftLimit; angle <= rightLimit; angle += secondScanStep) {
    moveServoSmooth(motorBase, angle, secondScanStepDelay);

    float dGrip = readSonarStable(gripTrigPin, gripEchoPin);
    Serial.print(F("Second Scan - base="));
    Serial.print(angle);
    Serial.print(F(" deg, grip sonar="));
    Serial.print(dGrip);
    Serial.println(F(" cm"));

    if (dGrip > 0.0 && dGrip < secondDetectCM) {
      // First time we see something start timer
      if (gripDetectStart == 0) {
        gripDetectStart = millis();
        Serial.println(F("Grip sonar: object detected, starting hold timer..."));
      }

      // Check if object was visible long enough
      if (millis() - gripDetectStart >= gripDetectHoldMS) {
        secondDetectFound = true;
        secondDetectBaseAngle = angle;

        Serial.print(F(">>> SECOND SCAN: object CONFIRMED after "));
        Serial.print(gripDetectHoldMS);
        Serial.println(F(" ms"));
        Serial.print(F("    baseAngle=")); Serial.println(secondDetectBaseAngle);
        Serial.print(F("    distance="));  Serial.println(dGrip);
        break;  // stop scanning
      }
    } 
    else {
      // Lost sight of object → reset timer
      if (gripDetectStart != 0) {
        Serial.println(F("Grip sonar: object lost, resetting hold timer"));
      }
      gripDetectStart = 0;
    }
  }

  if (!secondDetectFound) {
    Serial.println(F("SECOND SCAN: no object found in this fine scan."));
  }
}


//  --PickUp Sequence--
// From the second detected base angle, lower arm, close gripper, and lift.
void pickupObject() {
  Serial.println(F("== PICKUP SEQUENCE =="));

  if (!secondDetectFound) {
    Serial.println(F("No second detection; cannot pick up. Returning to HOME."));
    movePoseSmooth(poseHome, 10);
    return;
  }

  // 1) Rotate base to final grip position
  moveServoSmooth(motorBase, secondDetectBaseAngle, 10);

  // 2) Slowly move shoulder & elbow down to approximate pick pose
  int downShoulder = currentAngle[motorShoulder] - 25; // move shoulder down
  int downElbow    = currentAngle[motorElbow] + 20;    // extend elbow
  if (downShoulder < 40) downShoulder = 40;
  if (downElbow > 180)   downElbow = 180;

  moveServoSmooth(motorShoulder, downShoulder, 20);
  moveServoSmooth(motorElbow,    downElbow,    20);

  // stop if grip sonar gets below pickApproachCM
  float dGrip = readSonarStable(gripTrigPin, gripEchoPin);
  Serial.print(F("During pickup, grip sonar="));
  Serial.print(dGrip);
  Serial.println(F(" cm"));

  // 3) Close gripper to grab object
  moveServoSmooth(motorGrip, gripCloseAngle, 10);
  delay(200);

  // 4) Lift object to a safe pose
  int liftPose[numMotors];
  for (int i = 0; i < numMotors; i++) {
    liftPose[i] = poseLift[i];
  }
  // keep base at secondDetectBaseAngle
  liftPose[motorBase] = secondDetectBaseAngle;
  movePoseSmooth(liftPose, 10);
}

//  --Place Sequence--
// Rotate base to placeBaseAngle, open gripper, then go home.
void placeObject() {
  Serial.println(F("== PLACE SEQUENCE =="));

  // 1) Rotate base to place angle
  moveServoSmooth(motorBase, placeBaseAngle, 10);

  // 2) Move arm into place pose
  movePoseSmooth(posePlace, 10);

  // 3) Open gripper (drop object)
  moveServoSmooth(motorGrip, gripOpenAngle, 10);
  delay(200);

  // 4) Return to HOME pose
  movePoseSmooth(poseHome, 10);

  // Reset detection flags
  firstDetectFound = false;
  secondDetectFound = false;
}

//  ==ARDUINO SETUP==

void setup() {
  Serial.begin(115200);
  Serial.println(F("Robotic Arm - Dual Sonar Control Starting..."));

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(servoFreq);
  delay(10);

  // Sonar pins
  pinMode(gripTrigPin, OUTPUT);
  pinMode(gripEchoPin, INPUT);
  pinMode(shoulderTrigPin, OUTPUT);
  pinMode(shoulderEchoPin, INPUT);

  // Initialize servos to home pose
  for (int i = 0; i < numMotors; i++) {
    setServoAngleInstant(i, currentAngle[i]);
  }
  movePoseSmooth(poseHome, 10);

  state = stateFirstScan;
}

//  ==Main Loop==

void loop() {
  // Periodic telemetry for Processing
  if (millis() - lastTelemetryMs >= telemetryIntervalMS) {
    lastTelemetryMs = millis();
    sendTelemetry();
  }
  switch (state) {
    case stateFirstScan:
      scanFirstPhase();
      if (firstDetectFound) {
        state = stateMoveReady;
      } else {
        // If nothing found, we try again.
        state = stateFirstScan;
      }
      break;

    case stateMoveReady:
      moveToReadyPosition();
      state = stateSecondScan;
      break;

    case stateSecondScan:
      scanSecondPhase();
      if (secondDetectFound) {
        state = statePickup;
      } else {
        // If nothing found in second scan, go back to first scan
        state = stateFirstScan;
      }
      break;

    case statePickup:
      pickupObject();
      state = statePlace;
      break;

    case statePlace:
      placeObject();
      // After placing, restart the whole cycle
      state = stateFirstScan;
      break;
  }

  // Small pause between state actions
  delay(100);
}

/*
  ==HOW IT WORKS==

1. First Scanning Phase (stateFirstScan)
   - Base sweeps from baseScanMin (0°) to baseScanMax (150°).
   - At each angle:
       - Shoulder sonar (sonarShoulder) is read using readSonarStable().
       - If distance is between firstDetectMinCM and firstDetectMaxCM,
         the code stores:
           - firstDetectBaseAngle  (base angle)
           - firstDetectDistanceCm (detected distance)
         and stops the scan.
   - If nothing is found, the state tries scanning again.

2. Move arm down to ready position (STATE_MOVE_READY)
   - Base rotates to firstDetectBaseAngle.
   - According to detected distance:
       near  (<18 cm)  => poseReadyNear
       mid   (<28 cm)  => poseReadyMid
       far   (else)    => poseReadyFar
   - The joints (wrist, elbow, shoulder) move to that pose.
   - Sets state to stateSecondScan.

3. Second Scanning Phase (stateSecondScan)
   - Base scans around the previously detected angle:
       center = firstDetectBaseAngle
       from (center - secondScanArc) to (center + secondScanArc)
   - At each step:
       - Grip sonar (sonarGrip) is read.
       - If distance < secondDetectCM, we assume the object
         is right under the gripper.
       - secondDetectBaseAngle is stored.
   - If found, state -> statePickUp.
   - If not found, state -> stateFirstScan (start over).

4. PickUp Sequence (statePickUp)
   - Base rotates to secondDetectBaseAngle (fine alignment).
   - Shoulder and elbow move "down" to approach the object.
   - Gripper closes (gripCloseAngle).
   - Arm moves to a lift pose (poseLift).
   - Then state -> statePlace.

5. Place Sequence (statePlace)
   - Base rotates to placeBaseAngle (180°).
   - Arm moves to posePlace.
   - Gripper opens to release the object.
   - Arm returns to poseHome.
   - Detection flags reset and state -> stateFirstScan.
*/
