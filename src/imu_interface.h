#pragma once

// IMU interface: the core balance loop talks to this interface, never
// to the MPU6050 directly. That keeps the core testable on a host and
// lets a different sensor replace the MPU6050 without touching the
// control code. Units: accel in g, gyro in degrees per second.

struct ImuReading {
    float accel_x = 0.0f;  // forward axis, g
    float accel_y = 0.0f;  // sideways axis, g
    float accel_z = 0.0f;  // up axis, g
    float gyro_x = 0.0f;   // roll rate, degrees per second
    float gyro_y = 0.0f;   // pitch rate about the wheel axle, deg/s
    float gyro_z = 0.0f;   // yaw rate, degrees per second
};

class IImu {
public:
    virtual ~IImu() = default;
    virtual bool begin() = 0;
    // Fills reading with the latest sample. Returns false on I2C error.
    virtual bool read(ImuReading& reading) = 0;
};
