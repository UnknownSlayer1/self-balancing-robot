#pragma once

// Motor driver interface: the balance loop talks to this interface,
// never to the TB6612FNG directly. Speeds are in [-255, 255], where
// positive means forward. Keeps the core testable on a host.

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;
    virtual void begin() = 0;
    virtual void set_speed(int left, int right) = 0;
    virtual void stop() = 0;
};
