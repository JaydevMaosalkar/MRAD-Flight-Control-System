UAV control system with PID tuning, sensor integration and embedded actuation logic.
# MRAD Flight Control System

This repository contains work related to my final-year Mechatronics Engineering project (MRAD – Manoeuvrable RC Aircraft Development).

The project focuses on developing a basic flight control system for an RC aircraft, with emphasis on control logic, sensor integration and actuator response rather than full autonomy.

## Overview
The system uses an MPU6050 IMU to obtain orientation data and applies control logic to stabilise and command control surfaces via servos. An ESC is also driven as a servo output during experimental testing.

The code in this repository represents an experimental version used for testing control behaviour, signal routing and actuator response.

## Key Features
- IMU-based orientation sensing (MPU6050)
- Servo and ESC control using PWM signals
- Control logic for roll/pitch response
- Experimental ESC integration treated as a servo
- Incremental testing approach for stability and response tuning

## Notes
This repository contains a working experimental version of the control code. Further refinement and tuning were performed during testing and evaluation phases.

Additional context, system diagrams and project background are available on my portfolio:
https://my-portfolio-v2-4ab.pages.dev/
