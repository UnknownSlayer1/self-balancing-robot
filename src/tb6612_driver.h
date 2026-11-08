#pragma once

// Tb6612Driver: drives two DC motors through a TB6612FNG dual H bridge.
// Arduino dependent, used only by main.cpp, never by the host tests.
//
// Why this chip over the older L298N: the TB6612FNG is a MOSFET bridge,
// so it drops about 0.2 V instead of the L298N bipolar drop of 2 to
// 3 V. On a 7.4 V battery that is the difference between the motors
// getting about 7 V or about 5 V, plus far less heat. It also has a
// standby pin that cuts all motor current when the robot has fallen.
//
// Wiring per channel: IN1/IN2 set direction, PWM sets speed.
//   IN1 HIGH, IN2 LOW  -> forward
//   IN1 LOW,  IN2 HIGH -> reverse
//   both LOW            -> brake to stop

#include <Arduino.h>

#include "motor_interface.h"

class Tb6612Driver : public IMotorDriver {
public:
    Tb6612Driver(uint8_t ain1, uint8_t ain2, uint8_t pwma, uint8_t bin1,
                 uint8_t bin2, uint8_t pwmb, uint8_t standby)
        : ain1_(ain1), ain2_(ain2), pwma_(pwma), bin1_(bin1), bin2_(bin2),
          pwmb_(pwmb), standby_(standby) {}

    void begin() override {
        pinMode(ain1_, OUTPUT);
        pinMode(ain2_, OUTPUT);
        pinMode(pwma_, OUTPUT);
        pinMode(bin1_, OUTPUT);
        pinMode(bin2_, OUTPUT);
        pinMode(pwmb_, OUTPUT);
        pinMode(standby_, OUTPUT);
        digitalWrite(standby_, HIGH);  // take the driver out of standby
        stop();
    }

    void set_speed(int left, int right) override {
        drive_motor(ain1_, ain2_, pwma_, left);
        drive_motor(bin1_, bin2_, pwmb_, right);
    }

    void stop() override {
        digitalWrite(ain1_, LOW);
        digitalWrite(ain2_, LOW);
        digitalWrite(bin1_, LOW);
        digitalWrite(bin2_, LOW);
        analogWrite(pwma_, 0);
        analogWrite(pwmb_, 0);
    }

private:
    void drive_motor(uint8_t in1, uint8_t in2, uint8_t pwm, int speed) {
        if (speed > 255) speed = 255;
        if (speed < -255) speed = -255;
        if (speed >= 0) {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            analogWrite(pwm, speed);
        } else {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            analogWrite(pwm, -speed);
        }
    }

    uint8_t ain1_, ain2_, pwma_, bin1_, bin2_, pwmb_, standby_;
};
