# Control Logic Overview

The flight controller reads PWM signals from an RC receiver and combines them
with IMU-based roll feedback to provide basic stabilisation.

Roll stabilisation is implemented using a simple PID-style control loop applied
to the aileron channel. Elevator and rudder channels are passed directly from
the receiver.

This version focuses on validating signal flow, timing behaviour and actuator
response during bench testing.
