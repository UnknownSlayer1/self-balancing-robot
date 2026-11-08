#pragma once

// Mpu6050Driver: reads accel and gyro data from an MPU6050 (GY-521
// module) over I2C. Arduino dependent, used only by main.cpp, never
// by the host tests.
//
// Setup: accel range +-2 g (16384 LSB per g), gyro range +-250 deg/s
// (131 LSB per deg/s). The sensor is read in one 14 byte burst starting
// at ACCEL_XOUT_H so accel, temperature, and gyro stay from the same
// sample instant.
//
// I2C pitfalls handled here: the device is woken from sleep by clearing
// the SLEEP bit in PWR_MGMT_1, reads use a repeated start (endTransmission
// false) so the bus is not released mid transaction, and read() returns
// false instead of stale data when the bus errors.

#include <Arduino.h>
#include <Wire.h>

#include "imu_interface.h"

class Mpu6050Driver : public IImu {
public:
    explicit Mpu6050Driver(TwoWire& wire = Wire, uint8_t address = 0x68)
        : wire_(wire), address_(address) {}

    bool begin() override {
        wire_.begin();
        // Wake up: clear SLEEP bit, select the gyro X clock.
        if (!write_register(kPwrMgmt1, 0x01)) return false;
        // Gyro +-250 deg/s, accel +-2 g.
        if (!write_register(kGyroConfig, 0x00)) return false;
        if (!write_register(kAccelConfig, 0x00)) return false;
        delay(100);
        return true;
    }

    bool read(ImuReading& reading) override {
        wire_.beginTransmission(address_);
        wire_.write(kAccelXoutH);
        if (wire_.endTransmission(false) != 0) return false;
        if (wire_.requestFrom(address_, static_cast<uint8_t>(14)) != 14) return false;

        auto read16 = [&]() -> int16_t {
            int16_t v = static_cast<int16_t>((wire_.read() << 8) | wire_.read());
            return v;
        };

        const int16_t ax = read16();
        const int16_t ay = read16();
        const int16_t az = read16();
        read16();  // temperature, unused here
        const int16_t gx = read16();
        const int16_t gy = read16();
        const int16_t gz = read16();

        reading.accel_x = static_cast<float>(ax) / 16384.0f;
        reading.accel_y = static_cast<float>(ay) / 16384.0f;
        reading.accel_z = static_cast<float>(az) / 16384.0f;
        reading.gyro_x = static_cast<float>(gx) / 131.0f;
        reading.gyro_y = static_cast<float>(gy) / 131.0f;
        reading.gyro_z = static_cast<float>(gz) / 131.0f;
        return true;
    }

private:
    static constexpr uint8_t kPwrMgmt1 = 0x6B;
    static constexpr uint8_t kGyroConfig = 0x1B;
    static constexpr uint8_t kAccelConfig = 0x1C;
    static constexpr uint8_t kAccelXoutH = 0x3B;

    bool write_register(uint8_t reg, uint8_t value) {
        wire_.beginTransmission(address_);
        wire_.write(reg);
        wire_.write(value);
        return wire_.endTransmission() == 0;
    }

    TwoWire& wire_;
    uint8_t address_;
};
