/*
 ================================================================
 FINAL SAFE MOTOR CONTROLLER
 Arduino Nano + 2 x BL-4815F + BLDC geared motors
 ================================================================

 MOTOR 1 / LEFT
   PWM / VSP -> D5
   DIR       -> D6
   EN        -> D7
   FG        -> D2

 MOTOR 2 / RIGHT
   PWM / VSP -> D10
   DIR       -> D9
   EN        -> D8
   FG        -> D3

 IMU (BNO055)
   SDA -> A4
   SCL -> A5
   (standard Nano hardware I2C pins -- no conflict with
   motor pins above)

 BATTERY MONITOR (ADS1115)
   SDA -> A4  (shared I2C bus with BNO055 -- different address,
   SCL -> A5   no conflict: ADS1115=0x48, BNO055=0x28)
   A0  -> voltage divider tap (R1=100k to 24V rail, R2=18k to GND)
   Divider ratio = (R1+R2)/R2 = 6.5556
   Current sensing (shunt across A1/A2) NOT implemented yet --
   voltage/percentage only, see BATTERY_* settings below.

 ------------------------------------------------
 BL-4815F ENABLE
 ------------------------------------------------
 EN LOW  = RUN
 EN HIGH = DISABLE / COAST

 ------------------------------------------------
 SERIAL
 ------------------------------------------------
 Baud = 115200

 Commands:

 M,left,right

 Examples:

 M,80,80
     forward

 M,-80,-80
     reverse

 M,60,120
     curved motion

 M,120,60
     curved motion

 M,0,0
     coast stop

 S
     stop

 E
     enable permission

 D
     disable everything

 Q
     print telemetry

 ------------------------------------------------
 OUTGOING (Nano -> Pi, unsolicited)
 ------------------------------------------------

 I,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
     IMU sample, sent automatically at IMU_PUBLISH_INTERVAL.
     qw..qz  = orientation quaternion (unitless)
     gx..gz  = angular velocity, rad/s
     ax..az  = linear acceleration, m/s^2

 B,voltage,percentage
     Battery sample, sent automatically at BATTERY_PUBLISH_INTERVAL.
     voltage    = pack voltage, volts (after divider math)
     percentage = 0-100, from a linear voltage->percent curve --
                  see BATTERY_MIN_V / BATTERY_MAX_V below. Rough
                  estimate only, not a real fuel-gauge curve.

 W,left_wheel_rpm,right_wheel_rpm
     Wheel odometry sample, sent automatically at
     WHEEL_PUBLISH_INTERVAL. Same wheel1RPM/wheel2RPM values shown
     in the human-readable telemetry line, just in parseable form.
     Positive = physical forward rotation (already corrected for
     this robot's motor wiring/direction calibration -- NOT raw PWM
     sign). Left/right order matches M1/M2 (left/right) throughout
     this file.

 ------------------------------------------------
 SAFETY
 ------------------------------------------------

 1. Releasing command:
       PWM -> 0
       EN  -> HIGH
       motor coasts

 2. Restart:
       Arduino checks FG speed
       waits until sufficiently slow
       then enables driver

 3. Direction reversal:
       driver is disabled first
       motor coasts
       reverse allowed only after safe FG speed

 4. Soft PWM ramp:
       protects against sudden current transient

 5. Communication timeout:
       if Pi disappears -> motors stop

 ================================================================
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_ADS1X15.h>


// ================================================================
// PINS
// ================================================================

#define M1_PWM 5
#define M1_DIR 6
#define M1_EN  7
#define M1_FG  2

#define M2_PWM 10
#define M2_DIR 9
#define M2_EN  8
#define M2_FG  3

#define RELAY_PIN 11


// ================================================================
// DRIVER SETTINGS
// ================================================================

const bool ENABLE_LEVEL  = LOW;
const bool DISABLE_LEVEL = HIGH;


// ================================================================
// RELAY SETTINGS
// ================================================================

/*
 Flip these two if your relay module is active-LOW
 (most cheap 1-channel relay boards are active-LOW --
 double check with a multimeter before first power-up).
*/

const bool RELAY_ON_LEVEL  = HIGH;
const bool RELAY_OFF_LEVEL = LOW;

/*
 relayMode:
   0 = OFF
   1 = ON, topic/serial controlled (subject to COMMAND_TIMEOUT failsafe)
   2 = ALWAYS ON (ignores COMMAND_TIMEOUT, stays on until explicit R,0)
*/

int relayMode = 0;


// ================================================================
// IMU (BNO055)
// ================================================================

/*
 Address is 0x28 by default, 0x29 if the ADR/COM3 pin is pulled
 high. If the sensor fails to initialize, this is the first
 thing to check.
*/

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool imuOK = false;

const unsigned long IMU_PUBLISH_INTERVAL = 50; // ms (~20 Hz)

unsigned long lastImuPublish = 0;


// ================================================================
// BATTERY MONITOR (ADS1115)
// ================================================================

/*
 Default I2C address 0x48 (ADDR pin to GND). Different range from
 the BNO055 (0x28/0x29) -- no collision on the shared bus.
*/

Adafruit_ADS1115 ads;

bool adsOK = false;

const unsigned long BATTERY_PUBLISH_INTERVAL = 1000; // ms (1 Hz --
                                                        // battery
                                                        // voltage
                                                        // doesn't
                                                        // need to be
                                                        // read fast)

unsigned long lastBatteryPublish = 0;


// ================================================================
// WHEEL ODOMETRY PUBLISH
// ================================================================

/*
 wheel1RPM/wheel2RPM are already computed every loop (see the FG
 tachometer processing above) for the human-readable telemetry
 line -- this just re-sends the same numbers in a parseable format
 at a steady rate, for whatever ROS-side node wants to integrate
 them into real odometry.
*/

const unsigned long WHEEL_PUBLISH_INTERVAL = 50; // ms (~20 Hz --
                                                   // matches
                                                   // IMU_PUBLISH_INTERVAL,
                                                   // reasonable rate
                                                   // for odometry
                                                   // integration)

unsigned long lastWheelPublish = 0;

/*
 Voltage divider: R1 (100k) from the 24V rail down to the ADC tap,
 R2 (18k) from the tap to GND. ADS1115 reads the tap voltage;
 multiply back up by (R1+R2)/R2 to get the real pack voltage.
*/

const float BATTERY_DIVIDER_R1 = 100000.0; // ohms
const float BATTERY_DIVIDER_R2 = 18000.0;  // ohms
const float BATTERY_DIVIDER_RATIO =
  (BATTERY_DIVIDER_R1 + BATTERY_DIVIDER_R2) / BATTERY_DIVIDER_R2;

/*
 Rough linear voltage -> percentage curve for a 7S Li-ion pack
 (7 cells in series, 4.2V/cell full -- the 4P doesn't affect
 voltage, only capacity). NOT a real fuel-gauge curve (lithium
 packs are non-linear, especially in the middle of their range) --
 good enough for "is this thing about to die" at a glance, not for
 precise SOC.

 BATTERY_MIN_V uses 3.0V/cell as the safe cutoff (protects cell
 longevity) rather than the more aggressive 2.5V/cell some packs
 allow -- raise/lower if your BMS uses a different cutoff.

 ADS1115 headroom check (done when this was set up): at 29.4V
 full pack, tap voltage through the 100k/18k divider is
 29.4 * (18k/118k) = 4.49V. ADS1115 VDD here is 5V, and its
 default PGA full-scale is +-6.144V, so this reads fine with
 headroom. If you ever change VDD to 3.3V or change the divider
 resistors, re-check this math before trusting new readings.
*/

const float BATTERY_MAX_V = 29.4; // 7S Li-ion, 4.2V/cell full
const float BATTERY_MIN_V = 21.0; // 7S Li-ion, 3.0V/cell cutoff


// ================================================================
// PHYSICAL FORWARD DIRECTION
// ================================================================

/*
 If W on your dashboard makes one wheel rotate backward,
 change ONLY that motor from HIGH to LOW.

 Example:

 const bool M1_FORWARD_LEVEL = HIGH;
 const bool M2_FORWARD_LEVEL = LOW;

*/

const bool M1_FORWARD_LEVEL = HIGH;
const bool M2_FORWARD_LEVEL = HIGH;


// ================================================================
// MOTOR / GEARBOX SETTINGS
// ================================================================

/*
 Motor:
 90BL-200-24-3000

 Rated motor speed:
 3000 rpm

 Gearbox output:
 approximately 100 rpm

 Ratio:
 approximately 30:1

 IMPORTANT:
 POLE_PAIRS = 4 is currently inferred from your FG measurements.
*/

const float POLE_PAIRS = 4.0;

const float GEAR_RATIO = 30.0;


// ================================================================
// SPEED SAMPLING
// ================================================================

const unsigned long SPEED_SAMPLE_INTERVAL = 100; // ms

const unsigned long TELEMETRY_INTERVAL = 200;    // ms


// ================================================================
// SAFE RESTART
// ================================================================

/*
 The driver will NOT be enabled while the motor is
 still rotating rapidly after a stop.

 Your normal PWM80 speed gave approximately 60 Hz.

 We wait until FG falls to approximately 10 Hz or below.
*/

const float SAFE_REENABLE_FG_HZ = 15.0;


// Minimum EN-off time before re-enable.
const unsigned long MIN_DISABLE_TIME = 120; // ms


// ================================================================
// PWM RAMP
// ================================================================

const unsigned long RAMP_INTERVAL = 20; // ms


// Acceleration
const int ACCEL_STEP = 4;


// Slower commanded deceleration while driver is running.
const int DECEL_STEP = 2;


// ================================================================
// SERIAL FAILSAFE
// ================================================================

/*
 Pi dashboard currently sends commands repeatedly while
 a key is held.

 If communication disappears for >1 second:
 motors disable automatically.
*/

const unsigned long COMMAND_TIMEOUT = 1000;


// ================================================================
// FG COUNTERS
// ================================================================

volatile unsigned long fg1Count = 0;
volatile unsigned long fg2Count = 0;


unsigned long previousFG1 = 0;
unsigned long previousFG2 = 0;


// ================================================================
// FG SPEED
// ================================================================

float fg1RawHz = 0.0;
float fg2RawHz = 0.0;


float fg1Hz = 0.0;
float fg2Hz = 0.0;


// ================================================================
// RPM
// ================================================================

float motor1RPM = 0.0;
float motor2RPM = 0.0;

float wheel1RPM = 0.0;
float wheel2RPM = 0.0;


// ================================================================
// TARGET COMMAND FROM PI
// ================================================================

int targetM1 = 0;
int targetM2 = 0;


// ================================================================
// ACTUAL PWM CURRENTLY APPLIED
// ================================================================

int actualM1 = 0;
int actualM2 = 0;


// ================================================================
// DRIVER STATE
// ================================================================

bool motor1Enabled = false;
bool motor2Enabled = false;


// Last actual physical/electrical direction.
//
// +1 forward
// -1 reverse
//
int motor1Direction = 1;
int motor2Direction = 1;


// ================================================================
// DISABLE TIMES
// ================================================================

unsigned long motor1DisabledAt = 0;
unsigned long motor2DisabledAt = 0;


// ================================================================
// TIMING
// ================================================================

unsigned long lastSpeedSample = 0;
unsigned long lastTelemetry = 0;
unsigned long lastRamp = 0;
unsigned long lastCommand = 0;


// ================================================================
// SERIAL BUFFER
// ================================================================

String serialBuffer;


// ================================================================
// FG INTERRUPTS
// ================================================================

void fg1ISR()
{
  fg1Count++;
}


void fg2ISR()
{
  fg2Count++;
}


// ================================================================
// IMMEDIATE MOTOR 1 COAST
// ================================================================

void coastMotor1()
{
  analogWrite(M1_PWM, 0);

  digitalWrite(
    M1_EN,
    DISABLE_LEVEL
  );

  actualM1 = 0;

  motor1Enabled = false;

  motor1DisabledAt = millis();
}


// ================================================================
// IMMEDIATE MOTOR 2 COAST
// ================================================================

void coastMotor2()
{
  analogWrite(M2_PWM, 0);

  digitalWrite(
    M2_EN,
    DISABLE_LEVEL
  );

  actualM2 = 0;

  motor2Enabled = false;

  motor2DisabledAt = millis();
}


// ================================================================
// COAST BOTH
// ================================================================

void coastBoth()
{
  coastMotor1();
  coastMotor2();
}


// ================================================================
// ENABLE MOTOR 1 SAFELY
// ================================================================

void enableMotor1(int direction)
{
  // -------------------------------
  // Set direction FIRST
  // -------------------------------

  bool dirLevel;

  if(direction > 0)
  {
    dirLevel = M1_FORWARD_LEVEL;
  }
  else
  {
    dirLevel = !M1_FORWARD_LEVEL;
  }


  digitalWrite(
    M1_DIR,
    dirLevel
  );


  // Start with zero PWM.
  analogWrite(
    M1_PWM,
    0
  );


  delayMicroseconds(100);


  // Enable LAST.
  digitalWrite(
    M1_EN,
    ENABLE_LEVEL
  );


  motor1Enabled = true;

  motor1Direction = direction;

  actualM1 = 0;
}


// ================================================================
// ENABLE MOTOR 2 SAFELY
// ================================================================

void enableMotor2(int direction)
{
  bool dirLevel;

  if(direction > 0)
  {
    dirLevel = M2_FORWARD_LEVEL;
  }
  else
  {
    dirLevel = !M2_FORWARD_LEVEL;
  }


  digitalWrite(
    M2_DIR,
    dirLevel
  );


  analogWrite(
    M2_PWM,
    0
  );


  delayMicroseconds(100);


  digitalWrite(
    M2_EN,
    ENABLE_LEVEL
  );


  motor2Enabled = true;

  motor2Direction = direction;

  actualM2 = 0;
}


// ================================================================
// UPDATE MOTOR 1
// ================================================================

void updateMotor1()
{
  // ============================================================
  // TARGET ZERO -> COAST
  // ============================================================

  if(targetM1 == 0)
  {
    if(
      motor1Enabled ||
      actualM1 != 0
    )
    {
      coastMotor1();
    }

    return;
  }


  // ============================================================
  // DESIRED DIRECTION
  // ============================================================

  int desiredDirection;

  if(targetM1 > 0)
  {
    desiredDirection = 1;
  }
  else
  {
    desiredDirection = -1;
  }


  // ============================================================
  // DIRECTION CHANGE WHILE ENABLED
  //
  // Disable immediately.
  // Do NOT reverse electrically while rotor is moving.
  // ============================================================

  if(
    motor1Enabled &&
    desiredDirection != motor1Direction
  )
  {
    coastMotor1();

    return;
  }


  // ============================================================
  // DRIVER CURRENTLY DISABLED
  // ============================================================

  if(!motor1Enabled)
  {
    // Motor still spinning too fast?
    if(
      fg1RawHz >
      SAFE_REENABLE_FG_HZ
    )
    {
      return;
    }


    // Minimum EN-off time
    if(
      millis() - motor1DisabledAt <
      MIN_DISABLE_TIME
    )
    {
      return;
    }


    enableMotor1(
      desiredDirection
    );

    return;
  }


  // ============================================================
  // SOFT PWM RAMP
  // ============================================================

  int desiredPWM =
    abs(targetM1);


  int currentPWM =
    abs(actualM1);


  // Accelerate
  if(
    currentPWM <
    desiredPWM
  )
  {
    currentPWM += ACCEL_STEP;


    if(
      currentPWM >
      desiredPWM
    )
    {
      currentPWM =
        desiredPWM;
    }
  }


  // Decelerate gently
  else if(
    currentPWM >
    desiredPWM
  )
  {
    currentPWM -= DECEL_STEP;


    if(
      currentPWM <
      desiredPWM
    )
    {
      currentPWM =
        desiredPWM;
    }
  }


  actualM1 =
    motor1Direction *
    currentPWM;


  analogWrite(
    M1_PWM,
    currentPWM
  );
}


// ================================================================
// UPDATE MOTOR 2
// ================================================================

void updateMotor2()
{
  if(targetM2 == 0)
  {
    if(
      motor2Enabled ||
      actualM2 != 0
    )
    {
      coastMotor2();
    }

    return;
  }


  int desiredDirection;

  if(targetM2 > 0)
  {
    desiredDirection = 1;
  }
  else
  {
    desiredDirection = -1;
  }


  // Direction change
  if(
    motor2Enabled &&
    desiredDirection != motor2Direction
  )
  {
    coastMotor2();

    return;
  }


  // Driver disabled
  if(!motor2Enabled)
  {
    if(
      fg2RawHz >
      SAFE_REENABLE_FG_HZ
    )
    {
      return;
    }


    if(
      millis() - motor2DisabledAt <
      MIN_DISABLE_TIME
    )
    {
      return;
    }


    enableMotor2(
      desiredDirection
    );

    return;
  }


  // Soft PWM ramp
  int desiredPWM =
    abs(targetM2);


  int currentPWM =
    abs(actualM2);


  if(
    currentPWM <
    desiredPWM
  )
  {
    currentPWM += ACCEL_STEP;


    if(
      currentPWM >
      desiredPWM
    )
    {
      currentPWM =
        desiredPWM;
    }
  }


  else if(
    currentPWM >
    desiredPWM
  )
  {
    currentPWM -= DECEL_STEP;


    if(
      currentPWM <
      desiredPWM
    )
    {
      currentPWM =
        desiredPWM;
    }
  }


  actualM2 =
    motor2Direction *
    currentPWM;


  analogWrite(
    M2_PWM,
    currentPWM
  );
}


// ================================================================
// UPDATE BOTH MOTORS
// ================================================================

void updateMotors()
{
  unsigned long now =
    millis();


  if(
    now - lastRamp <
    RAMP_INTERVAL
  )
  {
    return;
  }


  lastRamp = now;


  updateMotor1();

  updateMotor2();
}


// ================================================================
// FG SPEED CALCULATION
// ================================================================

void updateSpeed()
{
  unsigned long now =
    millis();


  unsigned long elapsed =
    now - lastSpeedSample;


  if(
    elapsed <
    SPEED_SAMPLE_INTERVAL
  )
  {
    return;
  }


  // ============================================================
  // SAFE COPY INTERRUPT VALUES
  // ============================================================

  unsigned long count1;
  unsigned long count2;


  noInterrupts();

  count1 = fg1Count;
  count2 = fg2Count;

  interrupts();


  // ============================================================
  // NEW PULSES
  // ============================================================

  unsigned long delta1 =
    count1 - previousFG1;


  unsigned long delta2 =
    count2 - previousFG2;


  // ============================================================
  // TIME
  // ============================================================

  float dt =
    elapsed / 1000.0;


  // ============================================================
  // RAW FG FREQUENCY
  // ============================================================

  fg1RawHz =
    delta1 / dt;


  fg2RawHz =
    delta2 / dt;


  // ============================================================
  // SIMPLE LOW-PASS FILTER
  //
  // Makes dashboard RPM smoother.
  // ============================================================

  const float alpha = 0.35;


  fg1Hz =
    alpha * fg1RawHz +
    (1.0 - alpha) * fg1Hz;


  fg2Hz =
    alpha * fg2RawHz +
    (1.0 - alpha) * fg2Hz;


  // ============================================================
  // MOTOR RPM
  //
  // RPM = FG Hz × 60 / pole pairs
  // ============================================================

  motor1RPM =
    fg1Hz *
    60.0 /
    POLE_PAIRS;


  motor2RPM =
    fg2Hz *
    60.0 /
    POLE_PAIRS;


  // ============================================================
  // APPLY ROTATION DIRECTION
  // ============================================================

  if(
    motor1Direction < 0
  )
  {
    motor1RPM =
      -motor1RPM;
  }


  if(
    motor2Direction < 0
  )
  {
    motor2RPM =
      -motor2RPM;
  }


  // ============================================================
  // GEARBOX / WHEEL RPM
  // ============================================================

  wheel1RPM =
    motor1RPM /
    GEAR_RATIO;


  wheel2RPM =
    motor2RPM /
    GEAR_RATIO;


  // ============================================================
  // SAVE
  // ============================================================

  previousFG1 =
    count1;


  previousFG2 =
    count2;


  lastSpeedSample =
    now;
}


// ================================================================
// IMU: READ + PUBLISH
// ================================================================

/*
 Sent as its own line, separate from printTelemetry()'s
 human-readable format, so the Pi-side parser can branch on the
 leading character ('I' vs the unlabeled M1/M2 telemetry line)
 without needing to touch the existing telemetry parsing.

 Format:
 I,qw,qx,qy,qz,gx,gy,gz,ax,ay,az

   qw..qz : orientation quaternion (unitless)
   gx..gz : angular velocity, rad/s
   ax..az : linear acceleration, m/s^2
*/

void publishImu()
{
  if(
    !imuOK
  )
  {
    return;
  }


  imu::Quaternion q =
    bno.getQuat();

  imu::Vector<3> gyro =
    bno.getVector(
      Adafruit_BNO055::VECTOR_GYROSCOPE
    );

  imu::Vector<3> accel =
    bno.getVector(
      Adafruit_BNO055::VECTOR_LINEARACCEL
    );


  Serial.print(
    "I,"
  );

  Serial.print(q.w(), 4);
  Serial.print(",");
  Serial.print(q.x(), 4);
  Serial.print(",");
  Serial.print(q.y(), 4);
  Serial.print(",");
  Serial.print(q.z(), 4);
  Serial.print(",");

  Serial.print(gyro.x() * SENSORS_DPS_TO_RADS, 4);
  Serial.print(",");
  Serial.print(gyro.y() * SENSORS_DPS_TO_RADS, 4);
  Serial.print(",");
  Serial.print(gyro.z() * SENSORS_DPS_TO_RADS, 4);
  Serial.print(",");

  Serial.print(accel.x(), 4);
  Serial.print(",");
  Serial.print(accel.y(), 4);
  Serial.print(",");
  Serial.println(accel.z(), 4);
}


// ================================================================
// BATTERY: READ + PUBLISH
// ================================================================

/*
 Sent as its own line, same pattern as publishImu() -- distinct
 leading character so the Pi-side parser can branch on it without
 touching existing telemetry/IMU parsing.

 Format:
 B,voltage,percentage
*/

void publishBattery()
{
  if(
    !adsOK
  )
  {
    return;
  }


  int16_t raw =
    ads.readADC_SingleEnded(0);

  float tapVoltage =
    ads.computeVolts(raw);

  float packVoltage =
    tapVoltage * BATTERY_DIVIDER_RATIO;


  float percentage =
    (packVoltage - BATTERY_MIN_V) /
    (BATTERY_MAX_V - BATTERY_MIN_V) *
    100.0;

  // Clamp -- the linear curve above can go outside 0-100 if the
  // pack is charged past BATTERY_MAX_V or has dropped below
  // BATTERY_MIN_V, neither of which is a real "over/under 100%".
  if(percentage > 100.0) { percentage = 100.0; }
  if(percentage < 0.0)   { percentage = 0.0;   }


  Serial.print(
    "B,"
  );

  Serial.print(packVoltage, 2);
  Serial.print(",");
  Serial.println(percentage, 1);
}


// ================================================================
// WHEEL ODOMETRY: PUBLISH
// ================================================================

/*
 Sent as its own line, same pattern as publishImu()/publishBattery()
 -- distinct leading character so the Pi-side parser can branch on
 it without touching existing parsing.

 Format:
 W,left_wheel_rpm,right_wheel_rpm

 No separate "read" step -- wheel1RPM/wheel2RPM are already kept
 current by the FG tachometer processing that runs every loop
 (needed for the human-readable telemetry line regardless of
 whether this function ever gets called).
*/

void publishWheelOdom()
{
  Serial.print(
    "W,"
  );

  Serial.print(wheel1RPM, 2);
  Serial.print(",");
  Serial.println(wheel2RPM, 2);
}


// ================================================================
// TELEMETRY
// ================================================================

void printTelemetry()
{
  /*
   Keep this format unchanged because your Pi dashboard
   already understands it.
  */


  Serial.print(
    "M1 | PWM="
  );

  Serial.print(
    actualM1
  );


  Serial.print(
    " | FG="
  );

  Serial.print(
    fg1Hz,
    1
  );

  Serial.print(
    "Hz | Motor="
  );

  Serial.print(
    motor1RPM,
    0
  );

  Serial.print(
    "rpm | Wheel="
  );

  Serial.print(
    wheel1RPM,
    1
  );


  Serial.print(
    "rpm     ||     "
  );


  Serial.print(
    "M2 | PWM="
  );

  Serial.print(
    actualM2
  );


  Serial.print(
    " | FG="
  );

  Serial.print(
    fg2Hz,
    1
  );

  Serial.print(
    "Hz | Motor="
  );

  Serial.print(
    motor2RPM,
    0
  );

  Serial.print(
    "rpm | Wheel="
  );

  Serial.print(
    wheel2RPM,
    1
  );


  Serial.print(
    "rpm"
  );

  Serial.print(
    "     ||     RELAY="
  );

  Serial.println(
    relayMode
  );
}


// ================================================================
// SET TARGET MOTORS
// ================================================================

void setTargets(
  int left,
  int right
)
{
  left =
    constrain(
      left,
      -255,
      255
    );


  right =
    constrain(
      right,
      -255,
      255
    );


  targetM1 =
    left;


  targetM2 =
    right;


  lastCommand =
    millis();
}


// ================================================================
// PROCESS SERIAL
// ================================================================

void processCommand(
  String command
)
{
  command.trim();


  // ============================================================
  // MOTOR COMMAND
  // M,left,right
  // ============================================================

  if(
    command.startsWith("M,")
  )
  {
    int comma =
      command.indexOf(
        ',',
        2
      );


    if(
      comma < 0
    )
    {
      Serial.println(
        "ERR,BAD_COMMAND"
      );

      return;
    }


    int left =
      command.substring(
        2,
        comma
      ).toInt();


    int right =
      command.substring(
        comma + 1
      ).toInt();


    setTargets(
      left,
      right
    );


    return;
  }


  // ============================================================
  // STOP
  // ============================================================

  if(
    command == "S" ||
    command == "s"
  )
  {
    targetM1 = 0;
    targetM2 = 0;

    coastBoth();

    lastCommand =
      millis();

    return;
  }


  // ============================================================
  // DISABLE
  // ============================================================

  if(
    command == "D" ||
    command == "d"
  )
  {
    targetM1 = 0;
    targetM2 = 0;

    coastBoth();

    lastCommand =
      millis();

    return;
  }


  // ============================================================
  // RELAY: ON (topic/serial controlled, subject to failsafe)
  // ============================================================

  if(
    command == "R,1"
  )
  {
    relayMode = 1;

    digitalWrite(
      RELAY_PIN,
      RELAY_ON_LEVEL
    );

    lastCommand =
      millis();

    return;
  }


  // ============================================================
  // RELAY: OFF
  // ============================================================

  if(
    command == "R,0"
  )
  {
    relayMode = 0;

    digitalWrite(
      RELAY_PIN,
      RELAY_OFF_LEVEL
    );

    lastCommand =
      millis();

    return;
  }


  // ============================================================
  // RELAY: ALWAYS ON
  //
  // Deliberately does NOT touch lastCommand -- this mode must
  // survive the COMMAND_TIMEOUT failsafe. Only "R,0" turns it off.
  // ============================================================

  if(
    command == "R,A" ||
    command == "R,a"
  )
  {
    relayMode = 2;

    digitalWrite(
      RELAY_PIN,
      RELAY_ON_LEVEL
    );

    return;
  }


  // ============================================================
  // TELEMETRY REQUEST
  // ============================================================

  if(
    command == "Q" ||
    command == "q"
  )
  {
    printTelemetry();

    return;
  }
}


// ================================================================
// READ SERIAL
// ================================================================

void readSerial()
{
  while(
    Serial.available()
  )
  {
    char c =
      Serial.read();


    if(
      c == '\n' ||
      c == '\r'
    )
    {
      if(
        serialBuffer.length() > 0
      )
      {
        processCommand(
          serialBuffer
        );


        serialBuffer =
          "";
      }
    }


    else
    {
      serialBuffer += c;


      // Protect Nano SRAM.
      if(
        serialBuffer.length() >
        40
      )
      {
        serialBuffer =
          "";
      }
    }
  }
}


// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(
    115200
  );


  serialBuffer.reserve(
    40
  );


  // ============================================================
  // OUTPUTS
  // ============================================================

  pinMode(
    M1_PWM,
    OUTPUT
  );

  pinMode(
    M1_DIR,
    OUTPUT
  );

  pinMode(
    M1_EN,
    OUTPUT
  );


  pinMode(
    M2_PWM,
    OUTPUT
  );

  pinMode(
    M2_DIR,
    OUTPUT
  );

  pinMode(
    M2_EN,
    OUTPUT
  );


  // ============================================================
  // FG INPUTS
  // ============================================================

  pinMode(
    M1_FG,
    INPUT_PULLUP
  );


  pinMode(
    M2_FG,
    INPUT_PULLUP
  );


  // ============================================================
  // START SAFE
  // ============================================================

  analogWrite(
    M1_PWM,
    0
  );


  analogWrite(
    M2_PWM,
    0
  );


  digitalWrite(
    M1_EN,
    DISABLE_LEVEL
  );


  digitalWrite(
    M2_EN,
    DISABLE_LEVEL
  );


  pinMode(
    RELAY_PIN,
    OUTPUT
  );

  digitalWrite(
    RELAY_PIN,
    RELAY_OFF_LEVEL
  );

  relayMode = 0;


  // ============================================================
  // IMU (BNO055)
  //
  // Non-fatal if it fails to init -- motor control must keep
  // working even with the IMU unplugged/faulty. imuOK gates
  // publishImu() so no garbage is sent if init failed.
  // ============================================================

  Wire.begin();

  if(
    bno.begin()
  )
  {
    imuOK = true;

    bno.setExtCrystalUse(
      true
    );

    Serial.println(
      "IMU: BNO055 OK"
    );
  }
  else
  {
    imuOK = false;

    Serial.println(
      "IMU: BNO055 NOT FOUND"
    );
  }


  // ============================================================
  // BATTERY MONITOR (ADS1115)
  //
  // Same non-fatal pattern as the IMU -- motor control must keep
  // working even if this sensor is unplugged/faulty.
  // ============================================================

  if(
    ads.begin()
  )
  {
    adsOK = true;

    Serial.println(
      "BATTERY: ADS1115 OK"
    );
  }
  else
  {
    adsOK = false;

    Serial.println(
      "BATTERY: ADS1115 NOT FOUND"
    );
  }


  motor1Enabled = false;
  motor2Enabled = false;


  motor1DisabledAt =
    millis();


  motor2DisabledAt =
    millis();


  // ============================================================
  // FG INTERRUPTS
  // ============================================================

  attachInterrupt(
    digitalPinToInterrupt(
      M1_FG
    ),
    fg1ISR,
    RISING
  );


  attachInterrupt(
    digitalPinToInterrupt(
      M2_FG
    ),
    fg2ISR,
    RISING
  );


  // ============================================================
  // D10 PWM FREQUENCY
  //
  // Match D10 approximately to D5 (~976 Hz).
  // ============================================================

  TCCR1A =
    _BV(WGM10);


  TCCR1B =
    _BV(WGM12) |
    _BV(CS11) |
    _BV(CS10);


  // ============================================================
  // TIMERS
  // ============================================================

  unsigned long now =
    millis();


  lastSpeedSample =
    now;


  lastTelemetry =
    now;


  lastRamp =
    now;


  lastCommand =
    now;


  lastImuPublish =
    now;


  lastBatteryPublish =
    now;


  lastWheelPublish =
    now;


  // ============================================================
  // READY
  // ============================================================

  Serial.println();

  Serial.println(
    "============================================"
  );

  Serial.println(
    "SAFE BL-4815F MOTOR CONTROLLER"
  );

  Serial.println(
    "Arduino Nano"
  );

  Serial.println(
    "============================================"
  );

  Serial.println(
    "EN-stop + FG restart protection + soft ramp"
  );

  Serial.println(
    "Baud: 115200"
  );

  Serial.println(
    "READY"
  );
}


// ================================================================
// LOOP
// ================================================================

void loop()
{
  // ============================================================
  // SERIAL
  // ============================================================

  readSerial();


  // ============================================================
  // FG / RPM
  // ============================================================

  updateSpeed();


  // ============================================================
  // MOTOR CONTROL
  // ============================================================

  updateMotors();


  unsigned long now =
    millis();


  // ============================================================
  // TELEMETRY
  // ============================================================

  if(
    now - lastTelemetry >=
    TELEMETRY_INTERVAL
  )
  {
    printTelemetry();

    lastTelemetry =
      now;
  }


  // ============================================================
  // IMU
  // ============================================================

  if(
    now - lastImuPublish >=
    IMU_PUBLISH_INTERVAL
  )
  {
    publishImu();

    lastImuPublish =
      now;
  }


  // ============================================================
  // BATTERY
  // ============================================================

  if(
    now - lastBatteryPublish >=
    BATTERY_PUBLISH_INTERVAL
  )
  {
    publishBattery();

    lastBatteryPublish =
      now;
  }


  // ============================================================
  // WHEEL ODOMETRY
  // ============================================================

  if(
    now - lastWheelPublish >=
    WHEEL_PUBLISH_INTERVAL
  )
  {
    publishWheelOdom();

    lastWheelPublish =
      now;
  }


  // ============================================================
  // COMMUNICATION FAILSAFE
  // ============================================================

  if(
    now - lastCommand >
    COMMAND_TIMEOUT
  )
  {
    if(
      targetM1 != 0 ||
      targetM2 != 0 ||
      relayMode == 1
    )
    {
      targetM1 = 0;
      targetM2 = 0;

      coastBoth();

      // Only drop the relay if it was in topic-controlled mode.
      // relayMode == 2 (ALWAYS ON) deliberately ignores this failsafe.
      if(
        relayMode == 1
      )
      {
        relayMode = 0;

        digitalWrite(
          RELAY_PIN,
          RELAY_OFF_LEVEL
        );
      }

      Serial.println(
        "FAILSAFE,STOP"
      );
    }
  }
}