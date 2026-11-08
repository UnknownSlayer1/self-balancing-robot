// Self balancing two wheel robot.
//
// Thin Arduino glue only. All control logic lives in the pure C++
// headers (tilt_estimator.h, pid_controller.h, motor_mixer.h,
// safety.h) so it can be unit tested on a host without hardware.
// The drivers talk to the hardware through small interfaces
// (IImu, IMotorDriver) so the core never touches Arduino APIs.
//
// Control loop, 100 Hz:
//   1. Read accel + gyro from the MPU6050.
//   2. Fuse them into a tilt angle with the complementary filter.
//   3. PID on tilt error (target 0 degrees, upright) gives a motor
//      command. Positive tilt means leaning forward, so the wheels
//      drive forward to get back under the robot.
//   4. If the fall detector trips, cut the motors and wait until the
//      robot is held upright again.

#include <Arduino.h>
#include <Wire.h>

#include "motor_mixer.h"
#include "mpu6050_driver.h"
#include "pid_controller.h"
#include "safety.h"
#include "tb6612_driver.h"
#include "tilt_estimator.h"

// Wiring. See README.md for the full wiring table.
constexpr uint8_t kSdaPin = 21;
constexpr uint8_t kSclPin = 22;
constexpr uint8_t kAin1 = 26;
constexpr uint8_t kAin2 = 27;
constexpr uint8_t kPwma = 25;
constexpr uint8_t kBin1 = 18;
constexpr uint8_t kBin2 = 19;
constexpr uint8_t kPwmb = 23;
constexpr uint8_t kStandby = 32;
constexpr uint8_t kLedPin = 2;

// Loop timing: 100 Hz, dt = 0.01 s. The complementary filter alpha
// and the PID gains were chosen for this rate; if you change the rate,
// retune. See README.md for the tuning procedure.
constexpr unsigned long kLoopPeriodUs = 10000;
constexpr float kDtSeconds = 0.01f;

// Starting PID gains. These balance a typical build with the parts in
// the README. Every build is a little different, so expect to tune.
// Kp holds the robot up, Kd damps the wobble, Ki fixes a steady lean.
constexpr float kKp = 22.0f;
constexpr float kKi = 1.2f;
constexpr float kKd = 1.8f;

// Small trim if the robot's balance point is not exactly 0 degrees
// because the battery sits slightly off center. Measure it during
// tuning and set it here, in degrees.
constexpr float kAngleTrimDeg = 0.0f;

Mpu6050Driver imu(Wire);
Tb6612Driver motors(kAin1, kAin2, kPwma, kBin1, kBin2, kPwmb, kStandby);
TiltEstimator estimator(0.98f);
PidController pid(kKp, kKi, kKd, -255.0f, 255.0f, 50.0f);
FallDetector fall_detector(45.0f, 10.0f);

unsigned long last_loop_us = 0;
unsigned long last_print_ms = 0;

void setup() {
    Serial.begin(115200);
    pinMode(kLedPin, OUTPUT);
    Wire.begin(kSdaPin, kSclPin);

    if (!imu.begin()) {
        Serial.println("MPU6050 not found. Check wiring and I2C address.");
        while (true) {
            digitalWrite(kLedPin, HIGH);
            delay(200);
            digitalWrite(kLedPin, LOW);
            delay(200);
        }
    }
    motors.begin();

    // Let the sensor settle, then zero the filter at the starting tilt.
    delay(500);
    ImuReading reading;
    if (imu.read(reading)) {
        estimator.reset(0.0f);
        estimator.update(reading.accel_x, reading.accel_z, 0.0f, kDtSeconds);
    }

    Serial.println("Self balancing robot ready. Hold it upright and let go.");
    last_loop_us = micros();
}

void loop() {
    const unsigned long now_us = micros();
    if (now_us - last_loop_us < kLoopPeriodUs) {
        return;  // busy wait keeps the loop period exact
    }
    last_loop_us = now_us;

    ImuReading reading;
    if (!imu.read(reading)) {
        // I2C error: skip this cycle rather than acting on stale data.
        return;
    }

    // Gyro Y is the pitch rate about the wheel axle on this mounting.
    // If your robot balances the other way, flip the sign here.
    const float angle = estimator.update(reading.accel_x, reading.accel_z,
                                         reading.gyro_y, kDtSeconds);

    if (fall_detector.update(angle)) {
        motors.stop();
        digitalWrite(kLedPin, HIGH);
        return;
    }
    digitalWrite(kLedPin, LOW);

    // Error is measured tilt minus target. This is an inverted
    // pendulum, so the sign is the opposite of a normal servo loop:
    // leaning forward (positive angle) needs a positive, forward
    // motor command to drive the wheels back under the robot.
    const float error = angle - kAngleTrimDeg;
    const float output = pid.update(error, kDtSeconds);
    const MotorCommands cmd = MotorMixer::mix(output, 0.0f);
    motors.set_speed(cmd.left, cmd.right);

    // Telemetry at 10 Hz for tuning: angle, PID output, motor command.
    const unsigned long now_ms = millis();
    if (now_ms - last_print_ms >= 100) {
        last_print_ms = now_ms;
        Serial.print("angle=");
        Serial.print(angle, 2);
        Serial.print(" out=");
        Serial.print(output, 1);
        Serial.print(" left=");
        Serial.print(cmd.left);
        Serial.print(" right=");
        Serial.println(cmd.right);
    }
}
