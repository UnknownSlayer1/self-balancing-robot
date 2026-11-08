// Minimal Arduino API stub for host tests only.
//
// The pure C++ core (tilt_estimator.h, pid_controller.h,
// motor_mixer.h, safety.h) has no Arduino dependency and compiles
// without this file. This stub exists so test code that wants Arduino
// style helpers, and any future core code that uses millis(), can
// compile under g++ on a host. It is never used by the firmware build.

#pragma once

#include <cstdint>

using uint8_t = std::uint8_t;
using uint16_t = std::uint16_t;
using uint32_t = std::uint32_t;
using int16_t = std::int16_t;

inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
