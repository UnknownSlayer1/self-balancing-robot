#pragma once

// FallDetector: safety cutoff for the balance loop.
//
// If the tilt exceeds cutoff_deg the robot has fallen (or is about to)
// and the motors must stop immediately: a fallen robot with spinning
// wheels can damage itself, the floor, or a hand reaching for it.
// The cutoff latches until the robot is brought back near upright
// (below recover_deg), so it cannot restart while lying on its side.
//
// No Arduino dependency. Pure C++ so it can be unit tested on a host.

#include <cmath>

class FallDetector {
public:
    explicit FallDetector(float cutoff_deg = 45.0f, float recover_deg = 10.0f)
        : cutoff_deg_(cutoff_deg), recover_deg_(recover_deg), fallen_(false) {}

    void reset() { fallen_ = false; }

    // Returns true while the robot is in the fallen (motors off) state.
    bool update(float angle_deg) {
        if (!fallen_ && std::fabs(angle_deg) > cutoff_deg_) {
            fallen_ = true;
        } else if (fallen_ && std::fabs(angle_deg) < recover_deg_) {
            fallen_ = false;
        }
        return fallen_;
    }

    bool fallen() const { return fallen_; }

private:
    float cutoff_deg_;
    float recover_deg_;
    bool fallen_;
};
