//TEAM: Complexity Robotics
//Members: Wouter Wessels & Martin Ferreira
//Coach: Mr Paul Ntsinyi


#include <defines.h>
#include <QTRSensors.h>

// Line sensor properties
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// PID PROPERTIES
const double KP = 1.5;       // Constant P value
const double KD = 3.0;       // Constant D value
double lastError = 0;
const int GOAL = 1700;

// Motor properties
const int MAX_SPEED = 100;   // Maximum speed for PWM control (0-255)

// L298N H-Bridge Driver Pins (Reconfigured)
const int motorAPWM = 11;    // PWM pin for Motor A (Connect to ENA+ on L298N)
const int motorAIn1 = 12;    // Input 1 pin for Motor A (Connect to IN1 on L298N)
const int motorAIn2 = 13;    // Input 2 pin for Motor A (Connect to IN2 on L298N)
const int motorBPWM = 6;    // PWM pin for Motor B (Connect to ENB+ on L298N)
const int motorBIn3 = 3;    // Input 3 pin for Motor B (Connect to IN3 on L298N)
const int motorBIn4 = 7;    // Input 4 pin for Motor B (Connect to IN4 on L298N)989

// QTR sensor lib declarations
QTRSensors qtr;

void setup() {
  // QTR sensor config
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, 8, 9, 10}, SensorCount);

  // Motor control pins as outputs
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorAIn1, OUTPUT);
  pinMode(motorAIn2, OUTPUT);
  pinMode(motorBPWM, OUTPUT);
  pinMode(motorBIn3, OUTPUT);
  pinMode(motorBIn4, OUTPUT);

  // Calibration
  calibrateLineSensor();
}

void loop() {
  // Get line position & read line position
  uint16_t position = qtr.readLineWhite(sensorValues);

  // Compute error from line
  int error = position - GOAL;

  // Compute motor adjustment
  int adjustment = KP * error + KD * (error - lastError);

  // Store error for the next increment
  lastError = error;

  // Control the motors based on error and adjustment
  controlMotors(adjustment);
}

void controlMotors(int adjustment) {
  // Calculate left and right motor speeds
  int leftSpeed = MAX_SPEED - adjustment;  // Decrease left speed for positive error
  int rightSpeed = MAX_SPEED + adjustment; // Increase right speed for positive error

  // Ensure motor speeds are within bounds (0-255)
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  // Set motor directions and speeds
  digitalWrite(motorAIn1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(motorAIn2, leftSpeed < 0 ? HIGH : LOW);
  analogWrite(motorAPWM, abs(leftSpeed));

  // Reverse the motor directions for motor B
  digitalWrite(motorBIn3, rightSpeed < 0 ? HIGH : LOW);
  digitalWrite(motorBIn4, rightSpeed > 0 ? HIGH : LOW);
  analogWrite(motorBPWM, abs(rightSpeed));
}



void calibrateLineSensor() {
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on Arduino's LED to indicate we are in calibration mode

  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW); // Turn off Arduino's LED to indicate we are through with calibration
}
