# Control Logic Overview

The control system is structured around reading orientation data from the MPU6050 IMU and generating PWM outputs for control surfaces.

The control logic follows a simple structure:
1. Read raw accelerometer and gyroscope data
2. Estimate orientation angles
3. Apply proportional control logic for stability
4. Output PWM signals to servos and ESC

The primary goal at this stage was to validate signal flow, timing behaviour and actuator response rather than implement a full closed-loop autopilot.

Control gains were adjusted experimentally during bench testing.
