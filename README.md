# Self Balancing Two Wheel Robot

An ESP32 based self balancing robot, built in C++ with the Arduino framework and PlatformIO. It stands on two wheels using sensor fusion and a PID control loop, the same core ideas behind Segways, hoverboards, and rocket landing control.

## Why this matters for a CompE portfolio

This is not a blinking LED demo. It is a real time control system with real physics:

- **Control theory:** a PID loop stabilizes an inverted pendulum, a system that is unstable by nature. If the loop stops, the robot falls in under a second.
- **Sensor fusion:** a complementary filter combines a noisy accelerometer with a drifting gyroscope into one clean tilt estimate.
- **Real time embedded:** the control loop runs at a fixed 100 Hz. Timing jitter shows up directly as wobble, so loop timing is part of the design.
- **Hardware interfacing:** I2C sensor reads, PWM motor drive, power management, and a safety cutoff, all on a $20 parts budget.
- **Testable design:** the control core is pure C++ with no Arduino dependency, so it is unit tested on a host with `g++`. The hardware drivers sit behind small interfaces, so the core never touches hardware APIs.

## Parts list

Prices are estimates from AliExpress single unit listings checked Nov 8, 2026. Verify at checkout, prices are before shipping and can move. Search AliExpress with the search terms given.

| Part | Search term | Qty | Est. price |
|---|---|---|---|
| ESP32 DevKit V1 clone | ESP32 DevKit V1 | 1 | $4.00 |
| MPU6050 GY-521 module | MPU6050 GY-521 | 1 | $1.30 |
| TB6612FNG dual motor driver module | TB6612FNG motor driver | 1 | $1.50 |
| N20 6V DC gear motor with wheel | N20 gear motor wheel | 2 | $4.00 |
| 18650 battery holder, 2 cell series, plus 2x 18650 cells (or a 7.4V Li-ion pack) | 18650 battery holder 2 cell | 1 | $4.00 |
| Small chassis, acrylic plates, standoffs, screws | robot chassis acrylic | 1 | $3.00 |
| Breadboard, jumper wires, micro USB cable | breadboard jumper wire kit | 1 | $2.00 |
| **Total** | | | **$19.80** |

Itemized math: 4.00 + 1.30 + 1.50 + 4.00 + 4.00 + 3.00 + 2.00 = $19.80, so about $20.

Notes on the picks: the N20 motors should be the same gear ratio on both sides, around 100 RPM at 6V is a good starting point. Use protected 18650 cells or a pack with protection, and never short the battery leads. If you already own a USB cable or a breadboard, the total drops further.

## Wiring

Power the ESP32 from USB while building and testing. The battery powers the motors only. Tie all grounds together: ESP32 GND, TB6612FNG GND, battery negative, and MPU6050 GND must share one common ground, or the logic signals have no reference and nothing works.

### MPU6050 to ESP32

| MPU6050 pin | ESP32 pin |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| INT | Not connected |

Mount the MPU6050 flat, with its X axis pointing forward along the robot. The code reads gyro Y as the pitch rate about the wheel axle for this mounting. If your robot balances the other way, flip the gyro sign in `main.cpp` where noted.

### TB6612FNG to ESP32

| TB6612FNG pin | Connects to |
|---|---|
| VM | Battery positive, 7.4V |
| VCC | ESP32 3V3 (logic supply) |
| GND | Common ground |
| AIN1 | GPIO 26 |
| AIN2 | GPIO 27 |
| PWMA | GPIO 25 |
| BIN1 | GPIO 18 |
| BIN2 | GPIO 19 |
| PWMB | GPIO 23 |
| STBY | GPIO 32 |
| AO1 / AO2 | Left motor |
| BO1 / BO2 | Right motor |

### Motors and battery

| Part | Connects to |
|---|---|
| Left N20 motor | TB6612FNG AO1 and AO2 |
| Right N20 motor | TB6612FNG BO1 and BO2 |
| 2x 18650 in series, 7.4V | TB6612FNG VM and GND |

If a wheel spins backwards when the robot leans forward, swap that motor's two wires. Do not change the code to fix a swapped motor.

## How it works

```
MPU6050 accel + gyro
        |
        v
Complementary filter  ->  tilt angle in degrees
        |
        v
PID controller (error = tilt - target)  ->  motor command -255 to 255
        |
        v
Motor mixer  ->  left and right PWM
        |
        v
TB6612FNG  ->  N20 motors
```

The loop runs every 10 ms (100 Hz). Each cycle it reads the sensor, fuses accel and gyro into a tilt angle, runs the PID, and updates the motors. If the tilt passes 45 degrees, the fall detector cuts the motors and latches off until the robot is held near upright again.

File layout:

| File | What it is |
|---|---|
| `src/main.cpp` | Arduino glue only: setup, 100 Hz loop, telemetry |
| `src/tilt_estimator.h` | Complementary filter, pure C++ |
| `src/pid_controller.h` | PID with saturation and anti windup, pure C++ |
| `src/motor_mixer.h` | Throttle plus steering to left and right speeds, pure C++ |
| `src/safety.h` | Fall detector with latching cutoff, pure C++ |
| `src/imu_interface.h` | Sensor interface the core depends on |
| `src/motor_interface.h` | Motor driver interface the core depends on |
| `src/mpu6050_driver.h` | MPU6050 I2C driver, Arduino only |
| `src/tb6612_driver.h` | TB6612FNG PWM driver, Arduino only |
| `tests/test_main.cpp` | Host unit tests, run with `g++` |
| `platformio.ini` | ESP32 DevKit, Arduino framework |

## Hands on build

Do this in order. Each step is a checkpoint: stop and fix it before moving on.

1. **Flash a blink test.** Install PlatformIO, open this project, upload a blink sketch to the ESP32 first. If the board does not show up on USB, fix the driver and cable now, not after wiring.
2. **Wire the MPU6050 alone.** Connect VCC, GND, SDA, SCL only. Upload the firmware and watch the serial monitor at 115200 baud. You should see angle telemetry near 0 when the board is flat. Tilt the board by hand and watch the angle follow. If the firmware prints "MPU6050 not found", check SDA and SCL are not swapped, check the 3V3 supply, and check the address is 0x68 (AD0 low).
3. **Wire the motor driver with no motors.** Connect VM to the battery, VCC to 3V3, GND to common ground, and the seven control pins. Measure VM with a multimeter and confirm 7.4V at the driver before connecting motors.
4. **Connect one motor, test direction.** Hold the robot frame in the air, tilt it forward by hand, and confirm that motor drives forward. Then connect the second motor and confirm both drive the same way. Swap a motor's two wires if one runs backwards.
5. **Assemble the chassis.** Mount the motors low, the battery low and centered, and the ESP32 and MPU6050 high. Keep the center of mass as low as you can: a low battery makes tuning far easier. The MPU6050 must sit flat and rigid. Hot glue or tape that flexes will add vibration noise the filter has to fight.
6. **Balance the frame.** With power off, set the robot on its wheels and find where it balances by hand. If it always falls one way, shift the battery until it is close to neutral. Set `kAngleTrimDeg` in `main.cpp` to any small remaining offset in degrees.
7. **First balance attempt.** Hold the robot upright, power on, let go gently on a hard flat floor. Keep a hand near it. Expect it to fall. That is normal. Go to tuning below.

## PID tuning procedure

Starting gains in `main.cpp`: Kp 22, Ki 1.2, Kd 1.8. Tune in this order, one gain at a time, and watch the serial telemetry (angle, PID output, motor commands at 10 Hz).

1. **Set Ki and Kd to zero.** Raise Kp slowly until the robot oscillates steadily around upright instead of falling over. That oscillation means Kp is now strong enough to catch the fall. Note the value, then set Kp to about 60 percent of it.
2. **Add Kd.** Raise Kd in small steps until the oscillation damps out and the robot stands still with only small corrections. Too much Kd makes it jittery and noisy, because Kd amplifies sensor noise. Back off if it buzzes.
3. **Add Ki last.** If the robot stands but leans a few degrees to one side forever, raise Ki slowly until the lean goes away. Too much Ki causes slow hunting back and forth. A small Ki is enough here.
4. **Check the trim.** If it still leans after Ki is tuned, the frame is physically off balance. Move the battery, do not keep adding Ki to fix a mechanical problem.
5. **Retest after any mechanical change.** Moving the battery, changing wheels, or adding weight changes the tuning. Retune from step 1.

Common faults: robot falls instantly with no motor response means a wiring or direction fault, or the gyro sign is flipped. Robot oscillates violently means Kp or Kd too high. Robot leans and slowly drifts one way means Ki too low or the trim is off. Robot buzzes in place means Kd too high or the MPU6050 mount is loose.

## How to run the host tests

No hardware needed. From the project root:

```bash
./run_tests.sh
```

This compiles `tests/test_main.cpp` plus the pure C++ core with `g++ -std=c++17 -Wall -Wextra` and runs the tests. Expected result: 24 passed, 0 failed. The tests cover filter convergence and tracking from simulated accel and gyro data, PID saturation and closed loop step response, integral anti windup, motor mixer direction and saturation, and fall cutoff past 45 degrees.

## How to build the firmware

```bash
pio run -e default -t upload
pio device monitor -b 115200
```

Requires PlatformIO. The firmware was not built on a host without PlatformIO installed; run `pio run` once on your machine to confirm the Arduino build before wiring.

## Walkthrough

Read this before the mock interview questions. Be able to explain each point in your own words.

1. **The problem.** A two wheel robot is an inverted pendulum. Upright is an unstable equilibrium: any small tilt grows unless the wheels drive under the center of mass. The controller must sense tilt and react in milliseconds, forever.
2. **Sensing.** The MPU6050 gives acceleration and rotation rate. Neither alone is enough. The accelerometer measures gravity direction, so it gives absolute tilt, but it also picks up every bump and motor vibration. The gyroscope gives clean fast rotation rate, but integrating it accumulates drift, so after a minute it thinks upright is somewhere else.
3. **Fusion.** The complementary filter integrates the gyro each cycle, then nudges the result a little toward the accelerometer angle: `angle = 0.98 * (angle + gyro * dt) + 0.02 * accel_angle`. At 100 Hz the 0.98 weight puts the crossover near 0.5 Hz. Fast motion follows the gyro, slow drift follows gravity.
4. **Control.** The PID takes tilt error and outputs a motor command. Kp reacts to the current tilt, Ki removes a steady lean, Kd damps the swing by reacting to how fast the tilt is changing. Output is clamped to the PWM range of -255 to 255. The sign is positive feedback in the usual servo sense: leaning forward needs forward drive, because the wheels must get back under the robot.
5. **Actuation.** The TB6612FNG is a MOSFET H bridge. Two direction pins plus one PWM pin per motor set direction and speed. The mixer turns one throttle command into left and right speeds, with room for steering later.
6. **Safety.** Past 45 degrees the robot cannot recover, so the fall detector cuts the motors and latches off until the robot is held near upright. This protects the motors, the battery, and your fingers.
7. **Testing.** The filter, PID, mixer, and fall detector are pure C++ with no Arduino calls, so they compile and run under `g++` on a laptop. The drivers depend on small interfaces, so the core can be tested with simulated sensor data and a simulated plant before any hardware exists.

## Mock interview Q and A

Be able to answer these out loud, without reading the code.

**1. Why a complementary filter instead of a Kalman filter?**
A Kalman filter is optimal if you model the noise well, but it needs covariance tuning, matrix math, and more code and CPU for a benefit this robot does not need. The complementary filter is one line, runs in microseconds, has one intuitive tuning knob (alpha), and is provably good enough when one sensor is good fast and the other is good slow. I would use a Kalman filter if I had a full state model, GPS fusion, or strict accuracy requirements. For tilt at 100 Hz, the complementary filter is the right tradeoff.

**2. How did you tune the PID, and what does each term do?**
Kp first until steady oscillation, then back off to about 60 percent. Kd next to damp the oscillation. Ki last to remove steady lean. Kp reacts to present tilt, Ki to accumulated past error, Kd to the rate of change. I tuned on the real robot with serial telemetry, not by guessing, because the gains depend on mass, wheel size, and battery position.

**3. Why the TB6612FNG over the L298N?**
The L298N is a bipolar bridge that drops 2 to 3 V. On a 7.4 V battery the motors would see only about 5 V and the chip would get hot. The TB6612FNG is a MOSFET bridge with about a 0.2 V drop, so the motors get nearly the full battery voltage, it runs cooler, and it is smaller and cheaper. It also has a standby pin that cuts all motor current for the safety cutoff.

**4. Why does loop timing matter, and how do you keep it fixed?**
The filter integrates gyro rate times dt, and the PID derivative divides by dt. If the loop period jitters, both terms get the wrong dt and the estimate wobbles. The loop runs on a fixed 10 ms period using `micros()`, and dt is a constant 0.01 s. Sensor reads and serial prints are kept short so they cannot stretch a cycle. Telemetry is throttled to 10 Hz for the same reason.

**5. What goes wrong with I2C, and how did you handle it?**
Common faults: SDA and SCL swapped, missing common ground, wrong address (AD0 sets 0x68 or 0x69), and the sensor left in sleep mode after power on. The driver wakes the chip by clearing the SLEEP bit, uses a repeated start so the bus is not released mid read, reads all 14 bytes in one burst so accel and gyro come from the same sample, and returns false on bus error so the loop skips that cycle instead of acting on stale data.

**6. How does anti windup work, and why does this robot need it?**
When the robot is held tilted or has fallen, the error stays large and the integral term grows without bound. When the error finally reverses, that wound up integral keeps the motors saturated in the wrong direction for seconds. This controller uses conditional integration: it only accumulates the integral when the output is not saturated, or when the error would pull the output out of saturation. The integral is also hard clamped. Without it the robot overshoots badly after every large disturbance.

**7. How did you test this without hardware?**
The control core has no Arduino dependency, so it compiles under `g++` on a laptop. The tests feed the filter simulated accel and gyro data for a known tilt and check it converges, run the PID against a simulated plant and check the tilt goes to zero, check saturation and anti windup, check the mixer, and check the fall cutoff. Hardware dependent code is isolated behind the `IImu` and `IMotorDriver` interfaces, so the core never touches a sensor or a motor directly.
