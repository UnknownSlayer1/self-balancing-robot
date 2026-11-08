// Host unit tests for the self balancing robot core.
//
// Compiles the pure C++ core (tilt estimator, PID, motor mixer, fall
// detector) under g++ with no hardware and no Arduino libraries.
// Run with ./run_tests.sh
//
// The simulated sensor data uses a simple model: a robot tilted by
// angle A degrees sees gravity split across its axes as
//   accel_x = sin(A), accel_z = cos(A)
// in g, and a gyro rate equal to the rate of change of A.

#include <cmath>
#include <cstdio>

#include "../src/motor_mixer.h"
#include "../src/pid_controller.h"
#include "../src/safety.h"
#include "../src/tilt_estimator.h"

namespace {

int g_passed = 0;
int g_failed = 0;

void check(bool condition, const char* name) {
    if (condition) {
        ++g_passed;
        std::printf("PASS: %s\n", name);
    } else {
        ++g_failed;
        std::printf("FAIL: %s\n", name);
    }
}

bool near(float value, float expected, float tolerance) {
    return std::fabs(value - expected) <= tolerance;
}

constexpr float kPi = 3.14159265358979323846f;
float deg_to_rad(float deg) { return deg * kPi / 180.0f; }

// ---------------------------------------------------------------- tilt

void test_filter_converges_to_true_tilt() {
    // Robot held at a steady 10 degree tilt, gyro reads zero.
    TiltEstimator estimator(0.98f);
    const float dt = 0.01f;
    float angle = 0.0f;
    for (int i = 0; i < 500; ++i) {
        angle = estimator.update(std::sin(deg_to_rad(10.0f)),
                                 std::cos(deg_to_rad(10.0f)), 0.0f, dt);
    }
    check(near(angle, 10.0f, 0.5f),
          "filter converges to 10 deg tilt from accel alone");
}

void test_filter_tracks_moving_tilt() {
    // Robot tilts smoothly from 0 to 20 degrees over 2 seconds.
    // Accel follows the true angle, gyro reports the true rate.
    TiltEstimator estimator(0.98f);
    const float dt = 0.01f;
    const float rate_dps = 10.0f;  // 20 degrees in 2 seconds
    float angle = 0.0f;
    for (int i = 0; i < 200; ++i) {
        const float true_angle = rate_dps * dt * static_cast<float>(i);
        angle = estimator.update(std::sin(deg_to_rad(true_angle)),
                                 std::cos(deg_to_rad(true_angle)),
                                 rate_dps, dt);
    }
    check(near(angle, 20.0f, 2.0f),
          "filter tracks a 0 to 20 deg tilt driven by gyro + accel");
}

void test_filter_ignores_bad_dt() {
    TiltEstimator estimator;
    estimator.update(0.0f, 1.0f, 0.0f, 0.01f);
    const float before = estimator.angle();
    const float after = estimator.update(1.0f, 0.0f, 100.0f, 0.0f);
    check(after == before, "filter ignores update with dt <= 0");
}

void test_filter_reset() {
    TiltEstimator estimator;
    estimator.update(0.5f, 0.866f, 0.0f, 0.01f);
    estimator.reset(5.0f);
    check(near(estimator.angle(), 5.0f, 0.001f), "filter reset sets angle");
}

// ------------------------------------------------------------------ PID

void test_pid_saturates() {
    PidController pid(10.0f, 0.0f, 0.0f, -255.0f, 255.0f, 100.0f);
    const float out = pid.update(100.0f, 0.01f);
    check(near(out, 255.0f, 0.001f), "PID output saturates at +255");
    PidController pid_neg(10.0f, 0.0f, 0.0f, -255.0f, 255.0f, 100.0f);
    const float out_neg = pid_neg.update(-100.0f, 0.01f);
    check(near(out_neg, -255.0f, 0.001f), "PID output saturates at -255");
}

void test_pid_closed_loop_drives_error_to_zero() {
    // Closed loop sim: positive motor output drives the wheels forward,
    // which moves the base under the robot and reduces a positive tilt.
    // Plant: angle -= output * 0.002 each 10 ms step.
    PidController pid(8.0f, 0.2f, 0.5f, -255.0f, 255.0f, 50.0f);
    const float dt = 0.01f;
    float angle = 10.0f;
    for (int i = 0; i < 500; ++i) {
        const float output = pid.update(angle, dt);
        angle -= output * 0.002f;
    }
    check(near(angle, 0.0f, 0.5f),
          "PID closed loop drives 10 deg tilt to near zero");
}

void test_pid_anti_windup() {
    PidController pid(1.0f, 5.0f, 0.0f, -100.0f, 100.0f, 10.0f);
    const float dt = 0.01f;
    for (int i = 0; i < 200; ++i) {
        pid.update(100.0f, dt);  // huge error, output pinned at +100
    }
    check(pid.integral() <= 10.0f + 0.001f,
          "integral stays clamped at its limit under sustained error");
    // Reverse the error. With anti windup the output must come off the
    // positive rail quickly instead of staying pinned by a wound up
    // integral. Without it the integral would be near 200 here.
    float output = 0.0f;
    for (int i = 0; i < 20; ++i) {
        output = pid.update(-100.0f, dt);
    }
    check(output < 0.0f, "output unsaturates promptly after error reverses");
}

void test_pid_reset_clears_integral() {
    PidController pid(1.0f, 5.0f, 0.0f, -255.0f, 255.0f, 100.0f);
    for (int i = 0; i < 50; ++i) pid.update(10.0f, 0.01f);
    pid.reset();
    check(near(pid.integral(), 0.0f, 0.001f), "PID reset clears integral");
}

// -------------------------------------------------------------- mixer

void test_mixer() {
    MotorCommands c = MotorMixer::mix(100.0f, 0.0f);
    check(c.left == 100 && c.right == 100, "mixer: straight drive");

    c = MotorMixer::mix(100.0f, 50.0f);
    check(c.left == 150 && c.right == 50, "mixer: positive steer turns right");

    c = MotorMixer::mix(-100.0f, 0.0f);
    check(c.left == -100 && c.right == -100, "mixer: reverse drive");

    c = MotorMixer::mix(300.0f, 0.0f);
    check(c.left == 255 && c.right == 255, "mixer: saturates at +255");

    c = MotorMixer::mix(-300.0f, 0.0f);
    check(c.left == -255 && c.right == -255, "mixer: saturates at -255");

    c = MotorMixer::mix(200.0f, 100.0f);
    check(c.left == 255 && c.right == 100,
          "mixer: steering saturates one side only");
}

// -------------------------------------------------------------- safety

void test_fall_detector() {
    FallDetector detector(45.0f, 10.0f);
    check(!detector.update(20.0f), "no fall at 20 deg");
    check(!detector.update(44.0f), "no fall at 44 deg");
    check(detector.update(50.0f), "fall triggers beyond 45 deg");
    check(detector.update(50.0f), "fall stays latched");
    check(detector.update(20.0f), "fall stays latched at 20 deg");
    check(!detector.update(5.0f), "fall clears once back under 10 deg");

    FallDetector neg(45.0f, 10.0f);
    check(neg.update(-50.0f), "fall triggers on negative tilt too");
    check(!neg.update(-5.0f), "negative fall clears when upright");
}

}  // namespace

int main() {
    test_filter_converges_to_true_tilt();
    test_filter_tracks_moving_tilt();
    test_filter_ignores_bad_dt();
    test_filter_reset();
    test_pid_saturates();
    test_pid_closed_loop_drives_error_to_zero();
    test_pid_anti_windup();
    test_pid_reset_clears_integral();
    test_mixer();
    test_fall_detector();

    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
