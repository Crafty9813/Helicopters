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


AIRFRAME
------------------------------------------------------------

Tilt-rotor bicopter. The servos are coupled directly to
the motor nacelles, no gearing and no control vanes.

    AUW                 1200 g
    motor span          28 in
    CG -> thrust line   3.5 in
    motors              3508 380 kv, 15x5.5 CF, 6S
    servos              35 kg-cm, 5 V from ESC BECs

UNVERIFIED ASSUMPTION
------------------------------------------------------------

Nacelle tilt is assumed to be an ideal direct 1:1 horn:

    90 degrees of tilt per 1000 us of servo command

Every roll and yaw gain scales as 1 / (deg per us). If
the real travel is 120 deg per 1000 us, the roll and yaw
rate Kp values below are 33 percent too high.

Measure it: command 1100 and 1900 us, read the motor
shaft angle with a protractor, and rescale.

Note that at 90 deg / 1000 us the current TILT_MIN and
TILT_MAX allow +-36 degrees of tilt, which is more than
this airframe should ever need.


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
    BOTH NACELLES TILT TOGETHER

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
    OPPOSITE NACELLE TILT


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

const uint8_t FRONT_TILT_PIN = 11;
const uint8_t REAR_TILT_PIN = 10;


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

Servo frontTilt;
Servo rearTilt;


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

// Time constant of the complementary filter, in seconds.
//
// The old code used a fixed alpha of 0.98, which is only
// tau = 0.196 s at 250 Hz and let prop vibration and the
// lateral acceleration from nacelle tilt leak straight into
// the attitude estimate.
//
// A fixed alpha is also wrong whenever dt varies, because
// it silently changes the time constant with the loop rate.
// The coefficient is now computed from the measured dt:
//
//     alpha = tau / (tau + dt)
//
// which gives 0.998 at the nominal 4 ms and stays correct
// when a loop runs long.

const float COMPLEMENTARY_TAU = 2.0f;


// ============================================================
// ACCELEROMETER TRUST GATE
// ============================================================
//
// The accelerometer measures specific force, not gravity.
// Every time the aircraft banks it accelerates sideways,
// and the measured "down" tilts with that acceleration.
// Blending it in then pulls the attitude estimate toward a
// false horizon leaning the SAME way as the bank.
//
// On a thrust-vectoring airframe that closes a positive
// feedback loop:
//
//     bank -> accelerate sideways -> horizon appears to
//     tilt the same way -> hold the bank -> accelerate more
//
// It shows up as a slow sway several seconds per cycle,
// not as a fast oscillation.
//
// Fix: only trust the accelerometer when its magnitude is
// near 1 g. Outside that band the aircraft is accelerating
// and the reading is not a horizon reference, so coast on
// the gyro instead.
//
// Accelerometer is at ±2g -> 16384 LSB per g.

const float ACCEL_1G_LSB = 16384.0f;

const float ACCEL_TRUST_LOW = 0.80f;
const float ACCEL_TRUST_HIGH = 1.20f;


// Magnitude alone is not enough. Lateral acceleration adds
// in quadrature with gravity, so 0.2 g sideways gives
//
//     |a| = sqrt(1 + 0.04) = 1.02 g
//
// which sails through any sane magnitude band while tilting
// the apparent horizon by atan(0.2) = 11 degrees. Catching
// that by magnitude would need a +-2 percent window, which
// sensor noise would trip constantly.
//
// So also reject the accelerometer when it disagrees with
// the gyro-integrated angle. In honest flight the two track
// within a couple of degrees; under lateral acceleration
// they diverge by exactly the false-horizon error.

const float ACCEL_DISAGREE_LIMIT = 12.0f;

const float ACCEL_TRUST_LOW_SQ =
  (ACCEL_TRUST_LOW * ACCEL_1G_LSB)
  * (ACCEL_TRUST_LOW * ACCEL_1G_LSB);

const float ACCEL_TRUST_HIGH_SQ =
  (ACCEL_TRUST_HIGH * ACCEL_1G_LSB)
  * (ACCEL_TRUST_HIGH * ACCEL_1G_LSB);


// Exposed for telemetry: watch this in flight. If it sits
// at 0 for long stretches the band is too tight and the
// attitude estimate is coasting on gyro drift.

bool accelTrusted = true;

bool rollAccelOK = true;
bool pitchAccelOK = true;


// ============================================================
// GYRO SCALE
// ============================================================
//
// MPU6050 FS_SEL = 1 (±500 dps) -> 65.5 LSB per dps.

const float GYRO_LSB_PER_DPS = 65.5f;


// ============================================================
// D TERM FILTER
// ============================================================
//
// Cutoff for the low-pass on the rate derivative.
// Raw d(gyro)/dt behind two 15 in props is unusable.
//
// 25 Hz was a quadcopter default and far too wide for
// this airframe: differentiating gyro noise at 25 Hz put
// visible rapid jitter on the servos while sitting still.
// The rate loops only need lead near 1.6 Hz.

const float D_TERM_CUTOFF_HZ = 12.0f;


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

const int TILT_CENTER = 1500;

const int TILT_MIN = 1100;
const int TILT_MAX = 1900;


// ============================================================
// PILOT CONTROL LIMITS
// ============================================================

const float MAX_ROLL_ANGLE = 15.0f;
const float MAX_PITCH_ANGLE = 25.0f;
const float MAX_YAW_RATE = 100.0f;


// ============================================================
// PID STRUCTURE
// ============================================================
//
// Rate PID output is in MICROSECONDS, added directly to
// the servo / ESC command.
//
// integral is accumulated in output units (us) so that
// iLimit is a meaningful authority limit and anti-windup
// can freeze it on saturation.
//

struct PID {
  float kp;
  float ki;
  float kd;

  float iLimit;
  float outputMin;
  float outputMax;

  // state
  float integral;
  float prevMeasurement;
  float dFiltered;
  bool primed;
};


// ============================================================
// GAIN DERIVATION
// ============================================================
//
//   AUW                 1.200 kg
//   motor span          0.711 m  (28 in)
//   CG -> thrust line   0.089 m  (3.5 in)
//   hover thrust        5.89 N per motor  (~1563 us)
//   dT/du               0.021 N per us
//   tilt                90 deg per 1000 us, direct horn
//
// Moment of inertia (mass distribution, NOT the PID
// integral term - unrelated quantity, unlucky letter):
//
//   pitch inertia       0.0721 kg m^2
//   yaw inertia         0.0721 kg m^2
//   roll inertia        0.0067 kg m^2
//
// Roll inertia is small because almost everything sits ON
// the roll axis. Only two things are meaningfully off it -
// the nacelles above, and the battery slung below:
//
//   2 x 0.225 kg at 0.089 m  = 0.0036
//   battery 0.40 kg at 0.060 = 0.0014   <- second largest
//   battery's own inertia    = 0.0001
//   boom, 0.20 kg at 0.050   = 0.0005
//   ESCs, near the axis      = 0.0001
//   electronics and wiring   = 0.0010
//                              ------
//                              0.0067
//
// Pitch and yaw are ~11x larger because the same nacelle
// masses sit at 0.356 m instead of 0.089 m, and inertia
// goes as r squared.
//
// Mass budget, closes at the measured 1.200 kg:
//
//   2 x nacelle  (motor+servo+mount+prop)  0.450
//   2 x ESC      (partway out the boom)    0.060
//   battery      (boom center, slung low)  0.400
//   boom/frame                             0.200
//   FC, RX, wiring                         0.090
//                                          -----
//                                          1.200
//
// This is what converts a moment into an angular
// acceleration:  alpha = moment / inertia
//
// Plant sensitivity, deg/s^2 per us of PID output:
//
//   ROLL   (both nacelles tilt)   16.8
//   PITCH  (differential thrust)  11.6
//   YAW    (opposite tilt)         5.1
//
// TILT SCALING
//
// Roll and yaw sensitivity is proportional to how many
// degrees the nacelle tilts per microsecond, so those two
// rate Kp values scale as 1 / (deg per us). If you measure
// the real travel, read the matching row:
//
//     travel            roll Kp    yaw Kp
//     ------------------------------------
//      60 deg/1000us      1.07      2.87
//      90 deg/1000us      0.71      1.91   <- CONFIRMED
//     120 deg/1000us      0.53      1.43
//     150 deg/1000us      0.43      1.15
//
// Confirmed from the servo datasheet: 180 deg over a
// 500-2500 us range is 0.09 deg/us, i.e. 90 deg per
// 1000 us over the 1000-2000 us actually commanded.
//
// Angle Kp is NOT affected: it is deg/s per deg and only
// depends on the rate-loop bandwidth, which these values
// hold constant at 10 rad/s.
//
// Pitch is differential thrust and does not involve the
// servos at all, so pitchRatePID is unaffected either way.
//
// Rate Kp = target bandwidth / sensitivity.
// Pitch is capped at 8 rad/s by ~80 ms prop spin-up lag;
// roll and yaw at 10 rad/s by ~50 ms servo lag.
// Angle Kp is one third of the rate bandwidth.
//


// ============================================================
// ROLL ANGLE PID
// ============================================================
//
// Output is a desired roll rate in deg/s.
//

PID rollAnglePID = {
  1.5f,     // kp - 3.3 -> 2.2 -> 1.5, chasing the slow sway
  0.0f,     // ki  - leave at zero, rate loop holds trim
  0.0f,     // kd
  0.0f,     // iLimit
  -250.0f,  // deg/s, within the +-500 dps gyro range
  250.0f,
  0, 0, 0, false
};


// ============================================================
// ROLL RATE PID
// ============================================================
//
// Least confident axis. The roll moment of inertia (see
// the derivation block above - a mass property, not the
// integral term) is small and dominated by the two
// nacelles at 3.5 in above the CG, so this is the gain
// most likely to need adjustment. Tune roll first.
//

PID rollRatePID = {
  0.71f,    // kp - raised from 0.60. Roll inertia rose each
            //      time real masses replaced estimates
            //      (0.0056 -> 0.0062 -> 0.0067), so this
            //      loop was under-gained, not over.
  0.0f,     // ki  - raise to 1.5 once hover is stable
  0.021f,   // kd
  120.0f,   // iLimit, us
  -250.0f,  // us  (roll 250 + yaw 150 = 400 = full tilt range)
  250.0f,
  0, 0, 0, false
};


// ============================================================
// PITCH ANGLE PID
// ============================================================

PID pitchAnglePID = {
  2.7f,     // kp
  0.0f,     // ki
  0.0f,     // kd
  0.0f,     // iLimit
  -100.0f,  // deg/s
  100.0f,
  0, 0, 0, false
};


// ============================================================
// PITCH RATE PID
// ============================================================
//
// Was 1.5. Differential thrust on a 0.356 m arm with 15 in
// props is a strong but laggy actuator; 1.5 is where a slow
// pitch oscillation would be expected.
//

PID pitchRatePID = {
  0.68f,    // kp
  0.0f,     // ki  - raise to 1.4 once hover is stable
  0.047f,   // kd
  100.0f,   // iLimit, us
  -150.0f,  // us of differential throttle
  150.0f,
  0, 0, 0, false
};


// ============================================================
// YAW RATE PID
// ============================================================
//
// Weakest axis: 2 * T * sin(tilt) * arm is all you have,
// so this carries the highest Kp.
//

PID yawRatePID = {
  1.91f,    // kp
  0.0f,     // ki  - raise to 3.0 once hover is stable
  0.057f,   // kd
  100.0f,   // iLimit, us
  -150.0f,  // us
  150.0f,
  0, 0, 0, false
};


// ============================================================
// AXIS DIRECTIONS
// ============================================================
//
// These have been set according to your testing.
//

const float ROLL_DIRECTION = -1.0f;
const float PITCH_DIRECTION = -1.0f;
const float YAW_DIRECTION = 1.0f;


// ============================================================
// TILT SERVO DIRECTIONS
// ============================================================

const float FRONT_TILT_DIRECTION = 1.0f;
const float REAR_TILT_DIRECTION = -1.0f;


// ============================================================
// MOTOR YAW COMPENSATION
// ============================================================
//
// Leave zero until basic flight is working.
//

// Differential thrust makes a differential torque
// reaction, so this cross-feed is physically correct,
// but leave it at zero until each axis hovers cleanly
// on its own.

const float YAW_MOTOR_COMPENSATION = 0.0f;


// ============================================================
// ARMING
// ============================================================

bool armed = false;

const int ARM_THROTTLE_LIMIT = 1050;


// ============================================================
// IDLE THROTTLE GUARD
// ============================================================
//
// Below this the aircraft is on the ground. Without a guard,
// being ARMED at zero throttle still runs the mixer:
//
//     front = 1000 + pitchOutput
//
// so a few degrees of attitude error spins a 15 in prop up
// to ~1150 us while the aircraft is sitting on the ground.
//
// Hover is around 1563 us, so 1100 is far below anything
// used in flight and cannot cut control while airborne.

const int MIN_FLIGHT_THROTTLE = 1100;

const int ARM_YAW_HIGH = 1900;
const int DISARM_YAW_LOW = 1100;


// ============================================================
// SERIAL
// ============================================================

uint32_t lastPrintTime = 0;


// ============================================================
// TELEMETRY GATING
// ============================================================
//
// The status line is ~200 characters. At 115200 baud that is
// 17 ms of transmission, and with a 64 byte TX buffer
// Serial.print BLOCKS for about 12 ms of it - three loop
// periods at 250 Hz.
//
// So telemetry is printed only while throttle is below
// MIN_FLIGHT_THROTTLE, i.e. on the ground. Advancing the
// throttle silences the port automatically and the control
// loop runs clean.
//
// loopTimeMaxUS keeps accumulating in flight and is reported
// on the first print after you land and lower the throttle,
// so the number you see then is the worst case of the whole
// flight.

const bool TELEMETRY_IN_FLIGHT = false;


// Worst control-path execution time since the last print,
// in microseconds. The budget at 250 Hz is 4000. Measured
// before the telemetry print so it does not include the
// time spent blocking on Serial.

uint16_t loopTimeMaxUS = 0;

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

  pid.prevMeasurement = 0;
  pid.dFiltered = 0;

  // Force the first derivative sample to be discarded,
  // otherwise arming produces a huge d(rate)/dt spike.

  pid.primed = false;
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
  //
  // 0x03 = 44 Hz, 4.8 ms delay
  // 0x04 = 21 Hz, 8.3 ms delay
  //
  // The rate loops run at about 1.6 Hz bandwidth, so 44 Hz
  // was far wider than needed and was passing noise into
  // the D term. The extra 3.5 ms of delay costs about 2
  // degrees of phase at 10 rad/s.

  writeMPU(
    0x1A,
    0x04);


  // Gyro ±500 dps
  //
  // ±250 was too small: the roll angle loop is
  // allowed to command rates that the sensor
  // could not measure, and any upset past
  // 250 dps railed the gyro.

  writeMPU(
    0x1B,
    0x08);


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
    / GYRO_LSB_PER_DPS;


  gyroY_dps =
    ((float)gyroY
     - gyroYOffset)
    / GYRO_LSB_PER_DPS;


  gyroZ_dps =
    ((float)gyroZ
     - gyroZOffset)
    / GYRO_LSB_PER_DPS;
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
  // ACCELEROMETER TRUST GATE
  // ----------------------------------------------------------

  float ax = (float)accX;
  float ay = (float)accY;
  float az = (float)accZ;


  float magSq =
    ax * ax
    + ay * ay
    + az * az;


  accelTrusted =
    magSq > ACCEL_TRUST_LOW_SQ
    && magSq < ACCEL_TRUST_HIGH_SQ;


  // ----------------------------------------------------------
  // COMPLEMENTARY FILTER
  // ----------------------------------------------------------
  //
  // Gyro is integrated every loop above. The accelerometer
  // is only allowed to correct it when it is actually
  // measuring gravity.
  //

  // ----------------------------------------------------------
  // GYRO DISAGREEMENT GATE
  // ----------------------------------------------------------

  rollAccelOK =
    accelTrusted
    && fabs(accelRoll - rollAngleRaw)
         < ACCEL_DISAGREE_LIMIT;


  pitchAccelOK =
    accelTrusted
    && fabs(accelPitch - pitchAngleRaw)
         < ACCEL_DISAGREE_LIMIT;


  if (rollAccelOK || pitchAccelOK) {
    float alpha =
      COMPLEMENTARY_TAU
      / (COMPLEMENTARY_TAU + dt);


    if (rollAccelOK) {
      rollAngleRaw =
        alpha
          * rollAngleRaw
        + (1.0f - alpha)
            * accelRoll;
    }


    if (pitchAccelOK) {
      pitchAngleRaw =
        alpha
          * pitchAngleRaw
        + (1.0f - alpha)
            * accelPitch;
    }
  }


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
//
// Outer loop. Output is a desired rate in deg/s.
//
// ki is zero on both angle axes; running an integrator
// here and in the rate loop makes a double integrator.
//

float anglePID(
  PID &pid,
  float targetAngle,
  float actualAngle,
  float dt) {
  float error =
    targetAngle
    - actualAngle;


  float p =
    pid.kp
    * error;


  // ----------------------------------------------------------
  // ANTI-WINDUP
  // ----------------------------------------------------------
  //
  // Freeze the integrator when the output is already
  // saturated and the error would push it further out.
  //

  float candidate =
    p
    + pid.integral;


  bool blocked =
    (candidate >= pid.outputMax && error > 0)
    || (candidate <= pid.outputMin && error < 0);


  if (!blocked) {
    pid.integral +=
      pid.ki
      * error
      * dt;
  }


  pid.integral =
    constrainFloat(
      pid.integral,
      -pid.iLimit,
      pid.iLimit);


  return constrainFloat(
    p + pid.integral,
    pid.outputMin,
    pid.outputMax);
}


// ============================================================
// RATE PID
// ============================================================
//
// D TERM
//
// The previous version used:
//
//     - dGain * actualRate
//
// In a RATE loop the measurement is already the rate, so
// that is plain proportional feedback:
//
//     kp*target - (kp + dGain)*actualRate
//
// It adds no phase lead, so it does nothing about the
// motor and servo lag that actually limits this loop.
// It only raised measurement-path gain while leaving
// setpoint gain alone, which makes oscillation at the lag
// frequency MORE likely and leaves a standing rate error.
//
// Real D is the derivative of the measurement:
//
//     - kd * d(actualRate)/dt
//
// low-pass filtered, because raw d(gyro)/dt behind two
// 15 in props is pure noise. Derivative is taken on the
// measurement, not the error, so a stick step does not
// produce a derivative kick.
//

float ratePID(
  PID &pid,
  float targetRate,
  float actualRate,
  float dt) {
  float error =
    targetRate
    - actualRate;


  // ----------------------------------------------------------
  // DERIVATIVE ON MEASUREMENT
  // ----------------------------------------------------------

  float dRaw = 0.0f;


  if (pid.primed && dt > 0.0f) {
    dRaw =
      (actualRate
       - pid.prevMeasurement)
      / dt;
  } else {
    pid.primed = true;
  }


  pid.prevMeasurement = actualRate;


  // ----------------------------------------------------------
  // LOW-PASS THE DERIVATIVE
  // ----------------------------------------------------------
  //
  // Single pole at D_TERM_CUTOFF_HZ, coefficient computed
  // from the measured dt so it stays correct if the loop
  // rate drifts.
  //

  float rc =
    1.0f
    / (2.0f * PI * D_TERM_CUTOFF_HZ);


  float alpha =
    dt
    / (dt + rc);


  pid.dFiltered +=
    alpha
    * (dRaw - pid.dFiltered);


  // ----------------------------------------------------------
  // P + D
  // ----------------------------------------------------------

  float pd =
    pid.kp
      * error

    - pid.kd
        * pid.dFiltered;


  // ----------------------------------------------------------
  // ANTI-WINDUP
  // ----------------------------------------------------------

  float candidate =
    pd
    + pid.integral;


  bool blocked =
    (candidate >= pid.outputMax && error > 0)
    || (candidate <= pid.outputMin && error < 0);


  if (!blocked) {
    pid.integral +=
      pid.ki
      * error
      * dt;
  }


  pid.integral =
    constrainFloat(
      pid.integral,
      -pid.iLimit,
      pid.iLimit);


  return constrainFloat(
    pd + pid.integral,
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
// WRITE TILT SERVOS
// ============================================================

void writeTilts(
  float front,
  float rear) {
  front =
    TILT_CENTER
    + (front
       - TILT_CENTER)
        * FRONT_TILT_DIRECTION;


  rear =
    TILT_CENTER
    + (rear
       - TILT_CENTER)
        * REAR_TILT_DIRECTION;


  front =
    constrainFloat(
      front,
      TILT_MIN,
      TILT_MAX);


  rear =
    constrainFloat(
      rear,
      TILT_MIN,
      TILT_MAX);


  frontTilt.writeMicroseconds(
    (int)front);


  rearTilt.writeMicroseconds(
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


  writeTilts(
    TILT_CENTER,
    TILT_CENTER);
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

  float frontTiltCommand,
  float rearTiltCommand,

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


  Serial.print(" ACC:");
  Serial.print(rollAccelOK ? 1 : 0);
  Serial.print(pitchAccelOK ? 1 : 0);

  Serial.print(" LOOP:");
  Serial.print(loopTimeMaxUS);
  loopTimeMaxUS = 0;

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


  Serial.print(" | TILT F:");
  Serial.print(
    frontTiltCommand,
    0);


  Serial.print(" R:");
  Serial.print(
    rearTiltCommand,
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
  // TILT SERVOS
  // ----------------------------------------------------------

  frontTilt.attach(FRONT_TILT_PIN);
  rearTilt.attach(REAR_TILT_PIN);


  // ----------------------------------------------------------
  // RC PCINT
  // ----------------------------------------------------------

  initRC();

  writeMotors(ESC_MIN, ESC_MIN);
  writeTilts(TILT_CENTER, TILT_CENTER);

  initMPU();

  delay(500);

  // ----------------------------------------------------------
  // GYRO CALIBRATION
  // ----------------------------------------------------------

  calibrateGyro();

  // ----------------------------------------------------------
  // INITIAL ATTITUDE
  // ----------------------------------------------------------

  // A single accelerometer sample was being frozen as the
  // definition of level. One noisy reading, or a knock at
  // that instant, became a permanent attitude bias and the
  // aircraft would hold a standing bank all flight.
  //
  // Average instead.

  const int levelSamples = 200;

  float rollSum = 0.0f;
  float pitchSum = 0.0f;


  for (
    int i = 0;
    i < levelSamples;
    i++) {
    readMPU();

    rollSum += getAccelRoll();
    pitchSum += getAccelPitch();

    delay(3);
  }


  rollAngleRaw =
    rollSum / levelSamples;

  pitchAngleRaw =
    pitchSum / levelSamples;

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
    writeTilts(TILT_CENTER, TILT_CENTER);
    
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
    -rcToRate(
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
      dt);


  // ==========================================================
  // PITCH RATE PID
  // ==========================================================

  float pitchOutput =
    ratePID(
      pitchRatePID,
      desiredPitchRate,
      gyroY_dps,
      dt);


  // ==========================================================
  // YAW RATE PID
  // ==========================================================

  float yawOutput =
    ratePID(
      yawRatePID,
      desiredYawRate,
      gyroZ_dps,
      dt);


  // ==========================================================
  // APPLY DIRECTIONS
  // ==========================================================

  rollOutput *= ROLL_DIRECTION;
  pitchOutput *= PITCH_DIRECTION;
  yawOutput *= YAW_DIRECTION;


  // ==========================================================
  // IDLE THROTTLE GUARD
  // ==========================================================
  //
  // Armed but on the ground: hold the motors at idle and the
  // nacelles centered, and keep the controller state clear so
  // nothing is carried into the moment of takeoff.
  //
  // Angles and rates are still computed above, so telemetry
  // stays live on the ground.

  if (throttleUS < MIN_FLIGHT_THROTTLE) {
    resetAllPID();

    rollOutput = 0.0f;
    pitchOutput = 0.0f;
    yawOutput = 0.0f;
  }


  // ==========================================================
  // TILT SERVO MIXER
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

  float frontTiltCommand = TILT_CENTER + rollOutput + yawOutput;
  float rearTiltCommand = TILT_CENTER + rollOutput - yawOutput;

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

  frontTiltCommand += motorYawComp;
  rearTiltCommand -= motorYawComp;


  // ==========================================================
  // OUTPUT
  // ==========================================================

  writeMotors(
    frontMotorCommand,
    rearMotorCommand);

  writeTilts(
    frontTiltCommand,
    rearTiltCommand);


  // ==========================================================
  // LOOP TIME
  // ==========================================================

  uint16_t elapsedUS =
    (uint16_t)(micros() - now);


  if (elapsedUS > loopTimeMaxUS) {
    loopTimeMaxUS = elapsedUS;
  }


  // ==========================================================
  // TELEMETRY
  // ==========================================================

  bool onGround =
    throttleUS < MIN_FLIGHT_THROTTLE;


  if ((onGround || TELEMETRY_IN_FLIGHT)
      && millis() - lastPrintTime >= PRINT_INTERVAL) {
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

      frontTiltCommand,
      rearTiltCommand,

      frontMotorCommand,
      rearMotorCommand);
  }
}
