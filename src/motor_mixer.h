#pragma once

// MotorMixer: turns a balance (throttle) command and an optional
// steering command into left and right motor speeds.
//
// No Arduino dependency. Pure C++ so it can be unit tested on a host.
//
// Convention: positive throttle drives both wheels forward, positive
// steer turns the robot to the right by speeding up the left wheel and
// slowing the right wheel. Outputs are PWM duty values in [-255, 255].

#include <algorithm>

struct MotorCommands {
    int left;
    int right;
};

class MotorMixer {
public:
    static constexpr int kMaxSpeed = 255;

    static MotorCommands mix(float throttle, float steer = 0.0f) {
        MotorCommands out;
        out.left = clamp_speed(throttle + steer);
        out.right = clamp_speed(throttle - steer);
        return out;
    }

    // Single wheel helper, used by tests and by any code that drives
    // one motor directly.
    static int clamp_speed(float speed) {
        if (speed > static_cast<float>(kMaxSpeed)) return kMaxSpeed;
        if (speed < static_cast<float>(-kMaxSpeed)) return -kMaxSpeed;
        return static_cast<int>(speed);
    }
};
