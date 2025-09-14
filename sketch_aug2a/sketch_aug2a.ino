#include <ESP32Servo.h>
//this code is written by gemini
// Define the GPIO pins for our motors
#define MOTOR1_PIN 33 // Front-Right
#define MOTOR2_PIN 26 // Rear-Right
#define MOTOR3_PIN 27 // Rear-Left
#define MOTOR4_PIN 14 // Front-Left

// Create servo objects for each ESC
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  Serial.begin(115200);

  // Attach the ESCs to their respective pins
  motor1.attach(MOTOR1_PIN, 1000, 2000); // (pin, min pulse width, max pulse width)
  motor2.attach(MOTOR2_PIN, 1000, 2000);
  motor3.attach(MOTOR3_PIN, 1000, 2000);
  motor4.attach(MOTOR4_PIN, 1000, 2000);

  Serial.println("--- Drone Motor Test ---");
  Serial.println("Arming ESCs... sending min throttle.");
  
  // Send minimum throttle to arm the ESCs
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  
  delay(3000); // Wait 3 seconds for ESCs to arm
  Serial.println("ESCs should be armed now.");
  Serial.println("Testing motors one by one...");
}

void loop() {
  int throttle = 1150; // A very low throttle value for testing

  // Spin motor 1
  Serial.println("Motor 1 (Front-Right)");
  motor1.writeMicroseconds(throttle);
  delay(2000);
  motor1.writeMicroseconds(1000); // Stop motor 1
  delay(1000);

  // Spin motor 2
  Serial.println("Motor 2 (Rear-Right)");
  motor2.writeMicroseconds(throttle);
  delay(2000);
  motor2.writeMicroseconds(1000); // Stop motor 2
  delay(1000);

  // Spin motor 3
  Serial.println("Motor 3 (Rear-Left)");
  motor3.writeMicroseconds(throttle);
  delay(2000);
  motor3.writeMicroseconds(1000); // Stop motor 3
  delay(1000);

  // Spin motor 4
  Serial.println("Motor 4 (Front-Left)");
  motor4.writeMicroseconds(throttle);
  delay(2000);
  motor4.writeMicroseconds(1000); // Stop motor 4
  delay(5000); // Wait 5 seconds before repeating
}