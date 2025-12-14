/*
  MRAD Flight Controller - Main Aircraft Controller (V1)

  Description:
  Aircraft-side control code that reads RC receiver inputs,
  applies basic roll stabilisation using an MPU6050 IMU,
  and drives control surfaces and ESC outputs.

  Notes:
  - Roll stabilisation is applied only to the aileron channel
  - RC inputs are read as PWM pulses (1000–2000 µs)
  - ESC is treated as a servo output
  - Intended for experimental testing and tuning
*/

#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

// RC receiver channel input pins
#define CH1_PIN 2    // Aileron
#define CH2_PIN 3    // Elevator
#define CH3_PIN 4    // Rudder
#define CH4_PIN 5    // Throttle

// Servo output pins
#define AILERON_PIN  6
#define ELEVATOR_PIN 7
#define RUDDER_PIN   8
#define ESC_PIN      10

// Servo objects
Servo aileronServo;
Servo elevatorServo;
Servo rudderServo;
Servo esc;

// MPU6050 IMU
MPU6050 mpu;

// RC channel values (µs)
int ch1Value, ch2Value, ch3Value, ch4Value;

// IMU raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// PID parameters (roll)
float Kp = 2.0;
float Ki = 0.0;
float Kd = 1.0;

float error = 0.0;
float previousError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pidOutput = 0.0;

void setup() {
  Serial.begin(9600);

  // Attach servos and ESC
  aileronServo.attach(AILERON_PIN);
  elevatorServo.attach(ELEVATOR_PIN);
  rudderServo.attach(RUDDER_PIN);
  esc.attach(ESC_PIN);

  // Initialise I2C and IMU
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1) {
      delay(10);
    }
  }

  // Arm ESC with minimum throttle
  esc.writeMicroseconds(1000);
  delay(2000);
}

void loop() {
  // 1) Read RC receiver PWM inputs (timeout added for safety)
  ch1Value = pulseIn(CH1_PIN, HIGH, 25000);
  ch2Value = pulseIn(CH2_PIN, HIGH, 25000);
  ch3Value = pulseIn(CH3_PIN, HIGH, 25000);
  ch4Value = pulseIn(CH4_PIN, HIGH, 25000);

  // Validate RC input
  if (ch1Value < 900 || ch4Value < 900) {
    return; // skip loop if signal is lost
  }

  // 2) Read IMU data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 3) Compute roll angle from accelerometer (degrees)
  float roll = atan2((float)ay, (float)az) * 180.0 / PI;

  // 4) PID roll stabilisation (setpoint = 0°)
  error = -roll;
  integral += error;
  derivative = error - previousError;

  pidOutput = Kp * error
            + Ki * integral
            + Kd * derivative;

  previousError = error;

  // 5) Aileron command with stabilisation
  int aileronBase = map(ch1Value, 1000, 2000, 0, 180);
  int aileronCmd  = aileronBase + (int)pidOutput;
  aileronCmd = constrain(aileronCmd, 0, 180);
  aileronServo.write(aileronCmd);

  // 6) Direct elevator and rudder control
  int elevatorCmd = map(ch2Value, 1000, 2000, 0, 180);
  int rudderCmd   = map(ch3Value, 1000, 2000, 0, 180);

  elevatorServo.write(elevatorCmd);
  rudderServo.write(rudderCmd);

  // 7) Throttle to ESC
  int throttleCmd = constrain(ch4Value, 1000, 2000);
  esc.writeMicroseconds(throttleCmd);

  // 8) Serial logging (optional)
  Serial.print(millis() / 1000.0, 2);
  Serial.print(",");
  Serial.print(roll, 1);
  Serial.print(",");
  Serial.println(aileronCmd);

  delay(20); // ~50 Hz control loop
}
