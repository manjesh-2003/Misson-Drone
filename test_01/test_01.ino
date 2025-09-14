#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

//================================================================================
// Pin Definitions
//================================================================================
#define MOTOR1_PIN 33 // Front-Right (CCW)
#define MOTOR2_PIN 26 // Rear-Right  (CW)
#define MOTOR3_PIN 27 // Rear-Left   (CCW)
#define MOTOR4_PIN 14 // Front-Left  (CW)

//================================================================================
// Global Variables

//================================================================================
Adafruit_MPU6050 mpu;
Servo motor1, motor2, motor3, motor4;

// PID constants - These may need tuning for your specific drone
double Kp = 0.8;
double Ki = 0.005;
double Kd = 0.5;

// Variables for PID control
double pitch_error, roll_error;
double previous_pitch_error = 0, previous_roll_error = 0;
double pitch_integral = 0, roll_integral = 0;
double pitch_derivative, roll_derivative;
double pitch_correction, roll_correction;

// Sensor readings
float gyro_pitch, gyro_roll;
float acc_pitch, acc_roll;
float angle_pitch = 0, angle_roll = 0;

// Throttle and arming
int throttle = 1000; // Base throttle (motors off)
bool is_armed = false;

// Timing
unsigned long loop_timer;

//================================================================================
// Setup Function
//================================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Attach motors
  motor1.attach(MOTOR1_PIN, 1000, 2000);
  motor2.attach(MOTOR2_PIN, 1000, 2000);
  motor3.attach(MOTOR3_PIN, 1000, 2000);
  motor4.attach(MOTOR4_PIN, 1000, 2000);

  // Send initial off signal to ESCs
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(1000);

  loop_timer = micros(); // Initialize loop timer
}

//================================================================================
// Main Loop
//================================================================================
void loop() {
  // --- Read Sensor Data ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyro_roll = g.gyro.y;
  gyro_pitch = g.gyro.x;
  
  acc_roll = atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / PI;
  acc_pitch = atan(-a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI;

  // --- Complementary Filter for Stable Angles ---
  float dt = (micros() - loop_timer) / 1000000.0;
  loop_timer = micros();
  angle_roll = 0.98 * (angle_roll + gyro_roll * dt) + 0.02 * acc_roll;
  angle_pitch = 0.98 * (angle_pitch + gyro_pitch * dt) + 0.02 * acc_pitch;

  // --- Check for Arming/Disarming via Serial Monitor ---
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'a' && !is_armed) {
      is_armed = true;
      throttle = 1250; // Set a low hover throttle to start
      Serial.println("ARMED! Throttle at 1200. Send 'd' to disarm.");
    }
    if (command == 'd' && is_armed) {
      is_armed = false;
      throttle = 1000; // Turn motors off
      Serial.println("DISARMED. Motors OFF.");
    }
  }

  // --- PID Calculations ---
  roll_error = angle_roll;
  pitch_error = angle_pitch;
  
  roll_integral += roll_error * dt;
  pitch_integral += pitch_error * dt;

  roll_derivative = (roll_error - previous_roll_error) / dt;
  pitch_derivative = (pitch_error - previous_pitch_error) / dt;

  roll_correction = Kp * roll_error + Ki * roll_integral + Kd * roll_derivative;
  pitch_correction = Kp * pitch_error + Ki * pitch_integral + Kd * pitch_derivative;

  previous_roll_error = roll_error;
  previous_pitch_error = pitch_error;

  // --- Set Motor Speeds ---
  if (is_armed) {
    int motor1_speed = throttle - roll_correction + pitch_correction; // Front-Right
    int motor2_speed = throttle - roll_correction - pitch_correction; // Rear-Right
    int motor3_speed = throttle + roll_correction + pitch_correction; // Rear-Left
    int motor4_speed = throttle + roll_correction - pitch_correction; // Front-Left

    motor1_speed = constrain(motor1_speed, 1100, 1800);
    motor2_speed = constrain(motor2_speed, 1100, 1800);
    motor3_speed = constrain(motor3_speed, 1100, 1800);
    motor4_speed = constrain(motor4_speed, 1100, 1800);

    motor1.writeMicroseconds(motor1_speed);
    motor2.writeMicroseconds(motor2_speed);
    motor3.writeMicroseconds(motor3_speed);
    motor4.writeMicroseconds(motor4_speed);
  } else {
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
  }
}