#pragma once

// PidController: classic PID with output saturation and anti windup.
//
// No Arduino dependency. Pure C++ so it can be unit tested on a host.
//
// Anti windup strategy: conditional integration. The integral term is
// only accumulated when doing so would not push the output further into
// saturation. The integral is also hard clamped to integral_limit, so
// a long period of large error cannot build up a huge integral that
// then takes seconds to unwind once the error reverses.

#include <algorithm>

class PidController {
public:
    PidController(float kp, float ki, float kd, float out_min, float out_max,
                  float integral_limit)
        : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max),
          integral_limit_(integral_limit), integral_(0.0f), prev_error_(0.0f),
          last_output_(0.0f), first_run_(true) {}

    void reset() {
        integral_ = 0.0f;
        prev_error_ = 0.0f;
        last_output_ = 0.0f;
        first_run_ = true;
    }

    void set_gains(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    // error: setpoint minus measurement (degrees here). dt_s: seconds
    // since the previous call. Returns the control output, clamped to
    // [out_min, out_max].
    float update(float error, float dt_s) {
        if (dt_s <= 0.0f) {
            return last_output_;
        }

        float derivative = 0.0f;
        if (!first_run_) {
            derivative = (error - prev_error_) / dt_s;
        }
        first_run_ = false;

        // Tentative integral, clamped to its limit.
        float integral_candidate = integral_ + error * dt_s;
        integral_candidate = std::clamp(integral_candidate, -integral_limit_, integral_limit_);

        float output = kp_ * error + ki_ * integral_candidate + kd_ * derivative;
        float clamped = std::clamp(output, out_min_, out_max_);

        // Conditional integration: only accept the new integral if the
        // output is not saturated, or if the error would pull the output
        // back out of saturation.
        bool saturated = (clamped != output);
        bool error_fights_saturation =
            (output > out_max_ && error < 0.0f) || (output < out_min_ && error > 0.0f);
        if (!saturated || error_fights_saturation) {
            integral_ = integral_candidate;
        }

        prev_error_ = error;
        last_output_ = clamped;
        return clamped;
    }

    float integral() const { return integral_; }
    float last_output() const { return last_output_; }

private:
    float kp_, ki_, kd_;
    float out_min_, out_max_;
    float integral_limit_;
    float integral_;
    float prev_error_;
    float last_output_;
    bool first_run_;
};
