#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <PID_v1.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 (servo driver) initialization
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// MPU-6050 IMU sensor initialization
Adafruit_MPU6050 mpu;

// PID controller parameters for roll control
double setpoint = 0.0, input = 0.0, output = 0.0;
double Kp = 5.0, Ki = 0.1, Kd = 1.0;
PID rollPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Servo configuration (using PCA9685 channel 0 as example)
const int servoChannel = 0;
const int servoMin = 102;  // 1.0 ms pulse (approx)
const int servoMax = 512;  // 2.0 ms pulse (approx)

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Initialize IMU
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1) { delay(10); }
  }
  // Initialize PWM servo driver at 50 Hz
  pwm.begin();
  pwm.setPWMFreq(50);
  // Initialize PID: automatic mode, set output limits to ±45 degrees
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-45.0, 45.0);
}

void loop() {
  // Read accelerometer/gyro data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Compute roll angle via simple gyro integration (deg)
  static double rollAngle = 0.0;
  double gyroRate = g.gyro.x * 57.2958; // rad/s to deg/s
  double dt = 0.01; // loop time step
  rollAngle += gyroRate * dt;
  
  // Update PID input and compute new output
  input = rollAngle;
  setpoint = 0.0;           // desired roll = 0 (level)
  rollPID.Compute();
  
  // Map PID output (degrees) to servo PWM range
  double pwmSignal = map(output, -45, 45, servoMin, servoMax);
  pwm.setPWM(servoChannel, 0, int(pwmSignal));
  
  delay(10); // run loop ~100 Hz
}

