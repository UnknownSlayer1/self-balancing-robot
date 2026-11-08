#!/bin/bash
# Host tests for the self balancing robot core.
# Compiles the pure C++ core under g++ with no hardware and no Arduino
# libraries, then runs the test binary. Usage: ./run_tests.sh
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -Wall -Wextra -I tests/arduino_stub -I src \
    tests/test_main.cpp -o /tmp/self_balancing_robot_tests
/tmp/self_balancing_robot_tests
