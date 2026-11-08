#pragma once

// TiltEstimator: complementary filter fusing accelerometer and gyroscope
// data into a single tilt angle in degrees.
//
// No Arduino dependency. Pure C++ so it can be unit tested on a host.
//
// Idea: the gyroscope gives a fast, smooth rate of rotation but drifts
// over time. The accelerometer gives an absolute tilt reference from
// gravity but is noisy under vibration and acceleration. The filter
// trusts the gyro short term and nudges the estimate toward the
// accelerometer long term:
//   angle += gyro_rate * dt
//   angle = alpha * angle + (1 - alpha) * accel_angle
// With alpha = 0.98 at dt = 0.01 s the crossover is roughly 0.5 Hz:
// fast motion follows the gyro, slow drift follows gravity.

#include <cmath>

class TiltEstimator {
public:
    explicit TiltEstimator(float alpha = 0.98f) : alpha_(alpha), angle_deg_(0.0f) {}

    void reset(float initial_angle_deg = 0.0f) { angle_deg_ = initial_angle_deg; }

    // accel_x, accel_z: accelerometer readings in g, along the robot's
    // forward (x) and up (z) axes. gyro_rate_dps: rotation rate about
    // the wheel axle in degrees per second. dt_s: seconds since the
    // previous call. Returns the fused tilt angle in degrees, where
    // 0 means perfectly upright and positive means leaning forward.
    float update(float accel_x, float accel_z, float gyro_rate_dps, float dt_s) {
        if (dt_s <= 0.0f) {
            return angle_deg_;
        }
        // Tilt implied by gravity alone. atan2 handles the full range
        // and stays well behaved when accel_z is near zero.
        const float accel_angle_deg =
            std::atan2(accel_x, accel_z) * 180.0f / kPi;
        float angle = angle_deg_ + gyro_rate_dps * dt_s;
        angle = alpha_ * angle + (1.0f - alpha_) * accel_angle_deg;
        angle_deg_ = angle;
        return angle_deg_;
    }

    float angle() const { return angle_deg_; }

private:
    static constexpr float kPi = 3.14159265358979323846f;
    float alpha_;
    float angle_deg_;
};
