/*
============================================================
             BICOPTER FULL PID FLIGHT CONTROLLER
       Arduino Nano + MPU6050 + 2 ESC + 2 servos

        RC INPUT: PCINT INTERRUPT PWM CAPTURE

IMPORTANT:
    REMOVE PROPELLERS for all bench testing.

============================================================

RC INPUT PINS
------------------------------------------------------------

Arduino Nano / ATmega328P:

    D2 = Throttle = PCINT18
    D3 = Roll     = PCINT19
    D4 = Pitch    = PCINT20
    D7 = Yaw      = PCINT23

All four pins are handled by:

    PCINT2_vect

RC pulse widths are measured in the interrupt instead of
using pulseIn().

This means the main flight-control loop is not blocked while
waiting for RC pulses.


CONFIRMED SENSOR DIRECTIONS
------------------------------------------------------------

ROLL:
    Roll RIGHT = positive gyro X
    Roll LEFT  = negative gyro X

PITCH:
    Nose DOWN = positive gyro Y
    Nose UP   = negative gyro Y

PITCH CORRECTION:
    Nose DOWN:
        FRONT MOTOR FASTER
        REAR MOTOR SLOWER

    Nose UP:
        FRONT MOTOR SLOWER
        REAR MOTOR FASTER


CONTROL
------------------------------------------------------------

ROLL:
    RC roll
       ->
    desired roll angle
       ->
    roll angle P
       ->
    desired roll rate
       ->
    roll rate PID
       ->
    BOTH FINS

PITCH:
    RC pitch
       ->
    desired pitch angle
       ->
    pitch angle P
       ->
    desired pitch rate
       ->
    pitch rate PID
       ->
    DIFFERENTIAL MOTOR THRUST

YAW:
    RC yaw
       ->
    desired yaw rate
       ->
    yaw rate PID
       ->
    OPPOSITE FIN MOVEMENT


ARMING
------------------------------------------------------------

DISARMED:
    Throttle LOW + YAW RIGHT
        -> ARM

ARMED:
    Throttle LOW + YAW LEFT
        -> DISARM


OUTPUT
------------------------------------------------------------

Motors are ENABLED.

For first powered testing:

    PROPELLERS REMOVED.

============================================================
*/

#include <Wire.h>
#include <Servo.h>


// ============================================================
// PINS
// ============================================================

const uint8_t FRONT_ESC_PIN = 5;
const uint8_t REAR_ESC_PIN = 6;

const uint8_t FRONT_FIN_PIN = 11;
const uint8_t REAR_FIN_PIN = 10;


// ============================================================
// RC RECEIVER PINS
// ============================================================
//
// IMPORTANT:
// These pins are all on PORTD and PCINT2.
//
// D2 = PD2 = PCINT18
// D3 = PD3 = PCINT19
// D4 = PD4 = PCINT20
// D7 = PD7 = PCINT23
//

const uint8_t RC_THROTTLE_PIN = 2;
const uint8_t RC_ROLL_PIN = 3;
const uint8_t RC_PITCH_PIN = 4;
const uint8_t RC_YAW_PIN = 7;


// ============================================================
// MOTOR OUTPUT
// ============================================================
//
// TRUE = ESC commands are sent.
//
// Keep propellers removed during testing.
//

const bool ENABLE_MOTOR_OUTPUT = true;


// ============================================================
// ESC / SERVOS
// ============================================================

Servo frontESC;
Servo rearESC;

Servo frontFin;
Servo rearFin;


// ============================================================
// MPU6050
// ============================================================

const uint8_t MPU_ADDR = 0x68;

int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;

int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;


// ============================================================
// GYRO
// ============================================================

float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

float gyroX_dps = 0;
float gyroY_dps = 0;
float gyroZ_dps = 0;


// ============================================================
// ATTITUDE
// ============================================================

float rollAngle = 0;
float pitchAngle = 0;

// ============================================================
// STARTUP LEVEL REFERENCE
// ============================================================
//
// Whatever attitude the aircraft has during startup
// calibration becomes:
//
//     roll  = 0 degrees
//     pitch = 0 degrees
//
// Make sure the aircraft is sitting in the desired
// level position during startup.
//

float rollAngleRaw = 0;
float pitchAngleRaw = 0;

float rollLevelOffset = 0.0f;
float pitchLevelOffset = 0.0f;

const float COMPLEMENTARY_ALPHA = 0.98f;


// ============================================================
// LOOP TIMING
// ============================================================

const float LOOP_HZ = 250.0f;

const uint32_t LOOP_US =
  1000000UL / LOOP_HZ;

uint32_t lastLoopTime = 0;


// ============================================================
// RC VALUES
// ============================================================

int throttleUS = 1000;
int rollUS = 1500;
int pitchUS = 1500;
int yawUS = 1500;


// ============================================================
// RC FAILSAFE
// ============================================================

uint32_t lastRCUpdate = 0;

const uint32_t RC_FAILSAFE_TIMEOUT_MS = 100;


// ============================================================
// RC CENTER / DEAD BAND
// ============================================================

const int RC_CENTER = 1500;

const int RC_DEADBAND = 30;


// ============================================================
// RC VALID PULSE RANGE
// ============================================================

const uint16_t RC_MIN_VALID = 900;
const uint16_t RC_MAX_VALID = 2100;


// ============================================================
// PCINT RC VARIABLES
// ============================================================
//
// These are modified by the interrupt.
//
// volatile is required because the ISR changes them.
//

volatile uint16_t rcThrottleRaw = 1000;
volatile uint16_t rcRollRaw = 1500;
volatile uint16_t rcPitchRaw = 1500;
volatile uint16_t rcYawRaw = 1500;


// ============================================================
// RC RISING EDGE TIMESTAMPS
// ============================================================

volatile uint32_t rcThrottleStart = 0;
volatile uint32_t rcRollStart = 0;
volatile uint32_t rcPitchStart = 0;
volatile uint32_t rcYawStart = 0;


// ============================================================
// RC LAST VALID PULSE TIMESTAMPS
// ============================================================

volatile uint32_t rcThrottleLast = 0;
volatile uint32_t rcRollLast = 0;
volatile uint32_t rcPitchLast = 0;
volatile uint32_t rcYawLast = 0;


// ============================================================
// PREVIOUS PORT D STATE
// ============================================================

volatile uint8_t rcPreviousPortD = 0;


// ============================================================
// OUTPUT LIMITS
// ============================================================

const int ESC_MIN = 1000;
const int ESC_MAX = 2000;

const int FIN_CENTER = 1500;

const int FIN_MIN = 1100;
const int FIN_MAX = 1900;


// ============================================================
// PILOT CONTROL LIMITS
// ============================================================

const float MAX_ROLL_ANGLE = 35.0f;
const float MAX_PITCH_ANGLE = 25.0f;
const float MAX_YAW_RATE = 100.0f;


// ============================================================
// PID STRUCTURE
// ============================================================

struct PID {
  float kp;
  float ki;

  float integral;

  float outputMin;
  float outputMax;
};


// ============================================================
// ROLL ANGLE PID
// ============================================================

PID rollAnglePID = {
  5.0f,
  0.0f,
  0,
  -400.0f,
  400.0f
};


// ============================================================
// ROLL RATE PID
// ============================================================

PID rollRatePID = {
  1.0f,  // 1.5
  0.0f,
  0,
  -400.0f,
  400.0f
};


// ============================================================
// PITCH ANGLE PID
// ============================================================

PID pitchAnglePID = {
  4.0f,
  0.0f,
  0,
  -80.0f,
  80.0f
};


// ============================================================
// PITCH RATE PID
// ============================================================

PID pitchRatePID = {
  1.5f,
  0.0f,
  0,
  -100.0f,
  100.0f
};


// ============================================================
// YAW RATE PID
// ============================================================

PID yawRatePID = {
  1.0f,
  0.0f,
  0,
  -200.0f,
  200.0f
};


// ============================================================
// D GAINS
// ============================================================

float rollD = 0.04f;
float pitchD = 0.02f;
float yawD = 0.02f;


// ============================================================
// AXIS DIRECTIONS
// ============================================================
//
// These have been set according to your testing.
//

const float ROLL_DIRECTION = -1.0f;
const float PITCH_DIRECTION = 1.0f;
const float YAW_DIRECTION = 1.0f;


// ============================================================
// FIN DIRECTIONS
// ============================================================

const float FRONT_FIN_DIRECTION = 1.0f;
const float REAR_FIN_DIRECTION = -1.0f;


// ============================================================
// MOTOR YAW COMPENSATION
// ============================================================
//
// Leave zero until basic flight is working.
//

const float YAW_MOTOR_COMPENSATION = 0.02f;  // 0


// ============================================================
// PID INTEGRAL LIMIT
// ============================================================

const float PID_INTEGRAL_LIMIT = 50.0f;


// ============================================================
// ARMING
// ============================================================

bool armed = false;

const int ARM_THROTTLE_LIMIT = 1050;

const int ARM_YAW_HIGH = 1900;
const int DISARM_YAW_LOW = 1100;


// ============================================================
// SERIAL
// ============================================================

uint32_t lastPrintTime = 0;

const uint32_t PRINT_INTERVAL = 500;


// ============================================================
// UTILITY
// ============================================================

float constrainFloat(
  float value,
  float minimum,
  float maximum) {
  if (value < minimum)
    return minimum;

  if (value > maximum)
    return maximum;

  return value;
}


// ============================================================
// RESET PID
// ============================================================

void resetPID(PID &pid) {
  pid.integral = 0;
}


// ============================================================
// RESET ALL PID
// ============================================================

void resetAllPID() {
  resetPID(rollAnglePID);
  resetPID(rollRatePID);

  resetPID(pitchAnglePID);
  resetPID(pitchRatePID);

  resetPID(yawRatePID);
}


// ============================================================
// MPU WRITE
// ============================================================

void writeMPU(
  uint8_t reg,
  uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission();
}


// ============================================================
// MPU INITIALIZATION
// ============================================================

void initMPU() {
  // Wake MPU6050

  writeMPU(
    0x6B,
    0x00);

  delay(100);


  // Digital low-pass filter

  writeMPU(
    0x1A,
    0x03);


  // Gyro ±250 dps

  writeMPU(
    0x1B,
    0x00);


  // Accelerometer ±2g

  writeMPU(
    0x1C,
    0x00);


  // 250 Hz sample rate

  writeMPU(
    0x19,
    0x03);
}


// ============================================================
// MPU READ
// ============================================================

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(
    MPU_ADDR,
    (uint8_t)14);


  if (Wire.available() >= 14) {
    accX =
      Wire.read() << 8 | Wire.read();

    accY =
      Wire.read() << 8 | Wire.read();

    accZ =
      Wire.read() << 8 | Wire.read();


    // Temperature

    Wire.read();
    Wire.read();


    gyroX =
      Wire.read() << 8 | Wire.read();

    gyroY =
      Wire.read() << 8 | Wire.read();

    gyroZ =
      Wire.read() << 8 | Wire.read();
  }
}


// ============================================================
// GYRO CALIBRATION
// ============================================================

void calibrateGyro() {
  Serial.println();
  Serial.println("======================================");
  Serial.println("KEEP AIRCRAFT COMPLETELY STILL");
  Serial.println("GYRO CALIBRATION");
  Serial.println("======================================");

  delay(1000);

  const int samples = 1000;

  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;


  for (
    int i = 0;
    i < samples;
    i++) {
    readMPU();

    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;

    delay(2);
  }


  gyroXOffset =
    (float)sumX / samples;

  gyroYOffset =
    (float)sumY / samples;

  gyroZOffset =
    (float)sumZ / samples;


  Serial.println(
    "GYRO CALIBRATION COMPLETE");

  Serial.println();
}


// ============================================================
// GYRO UPDATE
// ============================================================

void updateGyro() {
  gyroX_dps =
    ((float)gyroX
     - gyroXOffset)
    / 131.0f;


  gyroY_dps =
    ((float)gyroY
     - gyroYOffset)
    / 131.0f;


  gyroZ_dps =
    ((float)gyroZ
     - gyroZOffset)
    / 131.0f;
}


// ============================================================
// ACCELEROMETER ROLL
// ============================================================

float getAccelRoll() {
  return atan2(
           (float)accY,
           (float)accZ)
         * 180.0f
         / PI;
}


// ============================================================
// ACCELEROMETER PITCH
// ============================================================

float getAccelPitch() {
  return atan2(
           -(float)accX,
           sqrt(
             (float)accY * accY
             + (float)accZ * accZ))
         * 180.0f
         / PI;
}


// ============================================================
// ATTITUDE UPDATE
// ============================================================

void updateAttitude(
  float dt) {
  float accelRoll =
    getAccelRoll();


  float accelPitch =
    getAccelPitch();


  // ----------------------------------------------------------
  // INTEGRATE GYRO
  // ----------------------------------------------------------

  rollAngleRaw +=
    gyroX_dps
    * dt;


  pitchAngleRaw +=
    gyroY_dps
    * dt;


  // ----------------------------------------------------------
  // COMPLEMENTARY FILTER
  // ----------------------------------------------------------

  rollAngleRaw =
    COMPLEMENTARY_ALPHA
      * rollAngleRaw
    + (1.0f - COMPLEMENTARY_ALPHA)
        * accelRoll;


  pitchAngleRaw =
    COMPLEMENTARY_ALPHA
      * pitchAngleRaw
    + (1.0f - COMPLEMENTARY_ALPHA)
        * accelPitch;


  // ----------------------------------------------------------
  // CONVERT TO STARTUP-REFERENCED ANGLES
  // ----------------------------------------------------------

  rollAngle =
    rollAngleRaw
    - rollLevelOffset;


  pitchAngle =
    pitchAngleRaw
    - pitchLevelOffset;
}

// ============================================================
// RC PCINT INTERRUPT
// ============================================================
//
// ATmega328P:
//
// PCINT2_vect = PORTD pin-change interrupt.
//
// D2 = PD2 = PCINT18 = throttle
// D3 = PD3 = PCINT19 = roll
// D4 = PD4 = PCINT20 = pitch
// D7 = PD7 = PCINT23 = yaw
//
// The ISR measures the HIGH pulse width.
//
// Typical RC PWM:
//
//     1000 us = minimum
//     1500 us = center
//     2000 us = maximum
//
// ============================================================

ISR(PCINT2_vect) {
  uint8_t currentPortD =
    PIND;


  uint8_t changed =
    currentPortD
    ^ rcPreviousPortD;


  // ==========================================================
  // THROTTLE - D2
  // ==========================================================

  if (changed & _BV(PD2)) {
    if (currentPortD & _BV(PD2)) {
      // Rising edge

      rcThrottleStart =
        micros();
    } else {
      // Falling edge

      uint32_t now =
        micros();


      uint32_t width =
        now
        - rcThrottleStart;


      if (
        width >= RC_MIN_VALID
        && width <= RC_MAX_VALID) {
        rcThrottleRaw =
          (uint16_t)width;

        rcThrottleLast =
          now;
      }
    }
  }


  // ==========================================================
  // ROLL - D3
  // ==========================================================

  if (changed & _BV(PD3)) {
    if (currentPortD & _BV(PD3)) {
      // Rising edge

      rcRollStart =
        micros();
    } else {
      // Falling edge

      uint32_t now =
        micros();


      uint32_t width =
        now
        - rcRollStart;


      if (
        width >= RC_MIN_VALID
        && width <= RC_MAX_VALID) {
        rcRollRaw =
          (uint16_t)width;

        rcRollLast =
          now;
      }
    }
  }


  // ==========================================================
  // PITCH - D4
  // ==========================================================

  if (changed & _BV(PD4)) {
    if (currentPortD & _BV(PD4)) {
      // Rising edge

      rcPitchStart =
        micros();
    } else {
      // Falling edge

      uint32_t now =
        micros();


      uint32_t width =
        now
        - rcPitchStart;


      if (
        width >= RC_MIN_VALID
        && width <= RC_MAX_VALID) {
        rcPitchRaw =
          (uint16_t)width;

        rcPitchLast =
          now;
      }
    }
  }


  // ==========================================================
  // YAW - D7
  // ==========================================================

  if (changed & _BV(PD7)) {
    if (currentPortD & _BV(PD7)) {
      // Rising edge

      rcYawStart =
        micros();
    } else {
      // Falling edge

      uint32_t now =
        micros();


      uint32_t width =
        now
        - rcYawStart;


      if (
        width >= RC_MIN_VALID
        && width <= RC_MAX_VALID) {
        rcYawRaw =
          (uint16_t)width;

        rcYawLast =
          now;
      }
    }
  }


  // Save current PORTD state

  rcPreviousPortD =
    currentPortD;
}


// ============================================================
// INITIALIZE RC PCINT
// ============================================================

void initRC() {
  // ----------------------------------------------------------
  // Configure RC pins
  // ----------------------------------------------------------

  pinMode(
    RC_THROTTLE_PIN,
    INPUT);

  pinMode(
    RC_ROLL_PIN,
    INPUT);

  pinMode(
    RC_PITCH_PIN,
    INPUT);

  pinMode(
    RC_YAW_PIN,
    INPUT);


  // ----------------------------------------------------------
  // Capture current PORTD state
  // ----------------------------------------------------------

  rcPreviousPortD =
    PIND;


  // ----------------------------------------------------------
  // Enable PCINT group 2
  //
  // PCIE2 = Pin Change Interrupt Enable 2
  // ----------------------------------------------------------

  PCICR |=
    _BV(PCIE2);


  // ----------------------------------------------------------
  // Enable individual PCINT pins
  //
  // D2 = PCINT18
  // D3 = PCINT19
  // D4 = PCINT20
  // D7 = PCINT23
  // ----------------------------------------------------------

  PCMSK2 |=
    _BV(PCINT18)
    | _BV(PCINT19)
    | _BV(PCINT20)
    | _BV(PCINT23);
}


// ============================================================
// RC UPDATE
// ============================================================
//
// Copies interrupt-generated values into the main program.
//
// The copies are protected because uint16_t is two bytes on
// the ATmega328P. Without disabling interrupts, an interrupt
// could occur between the two byte reads.
//

void updateRC() {
  uint16_t throttle;
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;


  uint32_t throttleLast;
  uint32_t rollLast;
  uint32_t pitchLast;
  uint32_t yawLast;


  // ----------------------------------------------------------
  // Atomic copy
  // ----------------------------------------------------------

  noInterrupts();


  throttle =
    rcThrottleRaw;

  roll =
    rcRollRaw;

  pitch =
    rcPitchRaw;

  yaw =
    rcYawRaw;


  throttleLast =
    rcThrottleLast;

  rollLast =
    rcRollLast;

  pitchLast =
    rcPitchLast;

  yawLast =
    rcYawLast;


  interrupts();


  // ----------------------------------------------------------
  // Copy values to main variables
  // ----------------------------------------------------------

  throttleUS =
    throttle;

  rollUS =
    roll;

  pitchUS =
    pitch;

  yawUS =
    yaw;


  // ----------------------------------------------------------
  // RC FAILSAFE
  //
  // EVERY channel must have received a valid pulse within
  // RC_FAILSAFE_TIMEOUT_MS.
  // ----------------------------------------------------------

  uint32_t nowUS =
    micros();


  bool throttleOK =
    ((uint32_t)(nowUS
                - throttleLast)
     < (RC_FAILSAFE_TIMEOUT_MS
        * 1000UL));


  bool rollOK =
    ((uint32_t)(nowUS
                - rollLast)
     < (RC_FAILSAFE_TIMEOUT_MS
        * 1000UL));


  bool pitchOK =
    ((uint32_t)(nowUS
                - pitchLast)
     < (RC_FAILSAFE_TIMEOUT_MS
        * 1000UL));


  bool yawOK =
    ((uint32_t)(nowUS
                - yawLast)
     < (RC_FAILSAFE_TIMEOUT_MS
        * 1000UL));


  if (
    throttleOK
    && rollOK
    && pitchOK
    && yawOK) {
    lastRCUpdate =
      millis();
  } else {
    // Force existing failsafe

    lastRCUpdate =
      millis()
      - RC_FAILSAFE_TIMEOUT_MS
      - 1;
  }
}


// ============================================================
// RC NORMALIZE
// ============================================================

float rcNormalized(
  int value) {
  int centered =
    value
    - RC_CENTER;


  if (
    abs(centered)
    < RC_DEADBAND) {
    centered = 0;
  }


  float result =
    (float)centered
    / 500.0f;


  return constrainFloat(
    result,
    -1.0f,
    1.0f);
}


// ============================================================
// RC TO ANGLE
// ============================================================

float rcToAngle(
  int value,
  float maximumAngle) {
  return rcNormalized(value)
         * maximumAngle;
}


// ============================================================
// RC TO RATE
// ============================================================

float rcToRate(
  int value,
  float maximumRate) {
  return rcNormalized(value)
         * maximumRate;
}


// ============================================================
// ANGLE PID
// ============================================================

float anglePID(
  PID &pid,
  float targetAngle,
  float actualAngle,
  float dt) {
  float error =
    targetAngle
    - actualAngle;


  pid.integral +=
    error
    * dt;


  pid.integral =
    constrainFloat(
      pid.integral,
      -PID_INTEGRAL_LIMIT,
      PID_INTEGRAL_LIMIT);


  float output =
    pid.kp
      * error

    + pid.ki
        * pid.integral;


  return constrainFloat(
    output,
    pid.outputMin,
    pid.outputMax);
}


// ============================================================
// RATE PID
// ============================================================
//
// D term uses actual gyro rate:
//
//     -D * actualRate
//
// ============================================================

float ratePID(
  PID &pid,
  float targetRate,
  float actualRate,
  float dGain,
  float dt) {
  float error =
    targetRate
    - actualRate;


  pid.integral +=
    error
    * dt;


  pid.integral =
    constrainFloat(
      pid.integral,
      -PID_INTEGRAL_LIMIT,
      PID_INTEGRAL_LIMIT);


  float output =
    pid.kp
      * error

    + pid.ki
        * pid.integral

    - dGain
        * actualRate;


  return constrainFloat(
    output,
    pid.outputMin,
    pid.outputMax);
}


// ============================================================
// WRITE MOTORS
// ============================================================

void writeMotors(
  float front,
  float rear) {
  front =
    constrainFloat(
      front,
      ESC_MIN,
      ESC_MAX);


  rear =
    constrainFloat(
      rear,
      ESC_MIN,
      ESC_MAX);


  if (ENABLE_MOTOR_OUTPUT) {
    frontESC.writeMicroseconds(
      (int)front);


    rearESC.writeMicroseconds(
      (int)rear);
  } else {
    frontESC.writeMicroseconds(
      ESC_MIN);


    rearESC.writeMicroseconds(
      ESC_MIN);
  }
}


// ============================================================
// WRITE FINS
// ============================================================

void writeFins(
  float front,
  float rear) {
  front =
    FIN_CENTER
    + (front
       - FIN_CENTER)
        * FRONT_FIN_DIRECTION;


  rear =
    FIN_CENTER
    + (rear
       - FIN_CENTER)
        * REAR_FIN_DIRECTION;


  front =
    constrainFloat(
      front,
      FIN_MIN,
      FIN_MAX);


  rear =
    constrainFloat(
      rear,
      FIN_MIN,
      FIN_MAX);


  frontFin.writeMicroseconds(
    (int)front);


  rearFin.writeMicroseconds(
    (int)rear);
}


// ============================================================
// DISARM
// ============================================================

void disarm() {
  armed = false;

  resetAllPID();


  writeMotors(
    ESC_MIN,
    ESC_MIN);


  writeFins(
    FIN_CENTER,
    FIN_CENTER);
}


// ============================================================
// ARM
// ============================================================

void arm() {
  armed = true;

  resetAllPID();


  Serial.println(
    "ARMED");
}


// ============================================================
// SERIAL STATUS
// ============================================================

void printStatus(
  float desiredRollAngle,
  float desiredRollRate,
  float rollOutput,

  float desiredPitchAngle,
  float desiredPitchRate,
  float pitchOutput,

  float desiredYawRate,
  float yawOutput,

  float frontFinCommand,
  float rearFinCommand,

  float frontMotorCommand,
  float rearMotorCommand) {
  Serial.print("ARM:");
  Serial.print(
    armed ? "YES" : "NO");


  Serial.print(" | RC T:");
  Serial.print(
    throttleUS);


  Serial.print(" R:");
  Serial.print(
    rollUS);


  Serial.print(" P:");
  Serial.print(
    pitchUS);


  Serial.print(" Y:");
  Serial.print(
    yawUS);


  Serial.print(" | ANGLE R:");
  Serial.print(
    rollAngle,
    1);


  Serial.print(" P:");
  Serial.print(
    pitchAngle,
    1);


  Serial.print(" | GYRO R:");
  Serial.print(
    gyroX_dps,
    1);


  Serial.print(" P:");
  Serial.print(
    gyroY_dps,
    1);


  Serial.print(" Y:");
  Serial.print(
    gyroZ_dps,
    1);


  Serial.print(" | Rcmd:");
  Serial.print(
    desiredRollAngle,
    1);


  Serial.print(" Rrate:");
  Serial.print(
    desiredRollRate,
    1);


  Serial.print(" Rout:");
  Serial.print(
    rollOutput,
    1);


  Serial.print(" | Pcmd:");
  Serial.print(
    desiredPitchAngle,
    1);


  Serial.print(" Prate:");
  Serial.print(
    desiredPitchRate,
    1);


  Serial.print(" Pout:");
  Serial.print(
    pitchOutput,
    1);


  Serial.print(" | Yrate:");
  Serial.print(
    desiredYawRate,
    1);


  Serial.print(" Yout:");
  Serial.print(
    yawOutput,
    1);


  Serial.print(" | FIN F:");
  Serial.print(
    frontFinCommand,
    0);


  Serial.print(" R:");
  Serial.print(
    rearFinCommand,
    0);


  Serial.print(" | MOTOR F:");
  Serial.print(frontMotorCommand, 0);


  Serial.print(" R:");
  Serial.print(rearMotorCommand, 0);


  Serial.println();
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);


  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin();

  Wire.setClock(400000);


  // ----------------------------------------------------------
  // ESC
  // ----------------------------------------------------------

  frontESC.attach(FRONT_ESC_PIN);

  rearESC.attach(REAR_ESC_PIN);


  // ----------------------------------------------------------
  // FINS
  // ----------------------------------------------------------

  frontFin.attach(FRONT_FIN_PIN);
  rearFin.attach(REAR_FIN_PIN);


  // ----------------------------------------------------------
  // RC PCINT
  // ----------------------------------------------------------

  initRC();

  writeMotors(ESC_MIN, ESC_MIN);
  writeFins(FIN_CENTER, FIN_CENTER);

  initMPU();

  delay(500);

  // ----------------------------------------------------------
  // GYRO CALIBRATION
  // ----------------------------------------------------------

  calibrateGyro();

  // ----------------------------------------------------------
  // INITIAL ATTITUDE
  // ----------------------------------------------------------

  readMPU();

  rollAngleRaw = getAccelRoll();
  pitchAngleRaw = getAccelPitch();

  // Save this as the level reference
  rollLevelOffset = rollAngleRaw;

  pitchLevelOffset = pitchAngleRaw;

  // Controller-relative attitude starts at zero

  rollAngle = 0.0f;

  pitchAngle = 0.0f;

  Serial.println();
  Serial.println("STARTUP LEVEL CALIBRATION");

  Serial.print("ROLL LEVEL OFFSET: ");

  Serial.println(rollLevelOffset, 2);

  Serial.print("PITCH LEVEL OFFSET: ");

  Serial.println(pitchLevelOffset, 2);

  Serial.println("STARTUP POSITION = LEVEL");

  Serial.println();


  // ----------------------------------------------------------
  // LOOP TIMER
  // ----------------------------------------------------------

  lastLoopTime = micros();

  // ----------------------------------------------------------
  // READY
  // ----------------------------------------------------------

  Serial.println();

  Serial.println("======================================");

  Serial.println("BICOPTER FULL PID CONTROLLER");

  Serial.println("======================================");

  Serial.println();

  Serial.println("RC INPUT: PCINT");

  Serial.println("D2 = THROTTLE");

  Serial.println("D3 = ROLL");

  Serial.println("D4 = PITCH");

  Serial.println("D7 = YAW");

  Serial.println();

  Serial.println("ROLL RIGHT = POSITIVE");

  Serial.println("ROLL LEFT  = NEGATIVE");

  Serial.println();

  Serial.println("NOSE DOWN = POSITIVE GYRO Y");

  Serial.println("NOSE UP   = NEGATIVE GYRO Y");

  Serial.println();

  Serial.println("PITCH CORRECTION:");

  Serial.println("NOSE DOWN -> FRONT MOTOR UP");

  Serial.println("NOSE DOWN -> REAR MOTOR DOWN");

  Serial.println();

  Serial.println("YAW RIGHT = POSITIVE");

  Serial.println("YAW LEFT  = NEGATIVE");

  Serial.println();

  Serial.println("MOTOR OUTPUT ENABLED");

  Serial.println();

  Serial.println("READY - DISARMED");

  Serial.println();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  // LOOP TIMING

  uint32_t now = micros();

  if ((uint32_t)(now - lastLoopTime) < LOOP_US) {
    return;
  }

  float dt = (now - lastLoopTime) / 1000000.0f;

  lastLoopTime = now;

  if (
    dt <= 0
    || dt > 0.02f) {
    dt = 0.004f;
  }

  updateRC();

  // RC FAILSAFE
  if (millis()  - lastRCUpdate > RC_FAILSAFE_TIMEOUT_MS) {
    if (armed) {
      Serial.println("RC FAILSAFE - DISARM");
    }

    disarm();

    return;
  }

  // ARM / DISARM
  if (throttleUS < ARM_THROTTLE_LIMIT) {
    // --------------------------------------------------------
    // DISARMED -> YAW RIGHT = ARM
    // --------------------------------------------------------

    if (
      !armed
      && yawUS > ARM_YAW_HIGH) {
      arm();
    }


    // --------------------------------------------------------
    // ARMED -> YAW LEFT = DISARM
    // --------------------------------------------------------
    if (armed && yawUS < DISARM_YAW_LOW) {
      disarm();
      return;
    }
  }

  readMPU();

  updateGyro();

  updateAttitude(dt);

  // ----------------------------------------------------------
  // DISARMED
  // ----------------------------------------------------------
  if (!armed) {
    writeMotors(ESC_MIN, ESC_MIN);
    writeFins(FIN_CENTER, FIN_CENTER);
    
    return;
  }


  // ==========================================================
  // ROLL OUTER LOOP
  // ==========================================================

  float desiredRollAngle =
    rcToAngle(
      rollUS,
      MAX_ROLL_ANGLE);


  float desiredRollRate =
    anglePID(
      rollAnglePID,
      desiredRollAngle,
      rollAngle,
      dt);


  // ==========================================================
  // PITCH OUTER LOOP
  // ==========================================================

  float desiredPitchAngle =
    rcToAngle(
      pitchUS,
      MAX_PITCH_ANGLE);


  float desiredPitchRate =
    anglePID(
      pitchAnglePID,
      desiredPitchAngle,
      pitchAngle,
      dt);


  // ==========================================================
  // YAW
  // ==========================================================

  float desiredYawRate =
    rcToRate(
      yawUS,
      MAX_YAW_RATE);


  // ==========================================================
  // ROLL RATE PID
  // ==========================================================

  float rollOutput =
    ratePID(
      rollRatePID,
      desiredRollRate,
      gyroX_dps,
      rollD,
      dt);


  // ==========================================================
  // PITCH RATE PID
  // ==========================================================

  float pitchOutput =
    ratePID(
      pitchRatePID,
      desiredPitchRate,
      gyroY_dps,
      pitchD,
      dt);


  // ==========================================================
  // YAW RATE PID
  // ==========================================================

  float yawOutput =
    ratePID(
      yawRatePID,
      desiredYawRate,
      gyroZ_dps,
      yawD,
      dt);


  // ==========================================================
  // APPLY DIRECTIONS
  // ==========================================================

  rollOutput *= ROLL_DIRECTION;
  pitchOutput *= PITCH_DIRECTION;
  yawOutput *= YAW_DIRECTION;


  // ==========================================================
  // FIN MIXER
  // ==========================================================
  //
  // Roll:
  //
  //     FRONT +
  //     REAR  +
  //
  // Yaw:
  //
  //     FRONT +
  //     REAR  -
  //
  // ==========================================================

  float frontFinCommand = FIN_CENTER + rollOutput + yawOutput;
  float rearFinCommand = FIN_CENTER + rollOutput - yawOutput;

  // ==========================================================
  // MOTOR MIXER
  // ==========================================================
  //
  // Positive pitch correction:
  //
  //     FRONT +
  //     REAR  -
  //
  // Therefore:
  //
  //     front = throttle + pitch
  //     rear  = throttle - pitch
  //
  // This matches the confirmed bench test.
  //
  // ==========================================================

  float baseThrottle = constrainFloat(throttleUS, ESC_MIN, ESC_MAX);

  float frontMotorCommand = baseThrottle + pitchOutput;
  float rearMotorCommand = baseThrottle - pitchOutput;


  // ==========================================================
  // MOTOR YAW COMPENSATION
  // ==========================================================

  float motorYawComp = pitchOutput * YAW_MOTOR_COMPENSATION;

  frontFinCommand += motorYawComp;
  rearFinCommand -= motorYawComp;


  // ==========================================================
  // OUTPUT
  // ==========================================================

  writeMotors(
    frontMotorCommand,
    rearMotorCommand);

  writeFins(
    frontFinCommand,
    rearFinCommand);

  // ==========================================================
  // TELEMETRY
  // ==========================================================

  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = millis();

    printStatus(
      desiredRollAngle,
      desiredRollRate,
      rollOutput,

      desiredPitchAngle,
      desiredPitchRate,
      pitchOutput,

      desiredYawRate,
      yawOutput,

      frontFinCommand,
      rearFinCommand,

      frontMotorCommand,
      rearMotorCommand);
  }
}
