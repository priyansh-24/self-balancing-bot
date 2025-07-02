// Required Libraries
#include <Wire.h>
#include <MPU6050_light.h>

// MPU6050 setup
MPU6050 mpu(Wire);

// Motor Pins
int motor1A = A2, motor1B = A3, enable1 = 6; // Motor 1 (left)
int motor2A = 4, motor2B = 9, enable2 = 5;  // Motor 2 (right)

// PID Constants
double Kp = 24;    // Proportional gain
double Ki = 200;   // Integral gain
double Kd = 1.2;   // Derivative gain

// System State
double targetAngle = 0;  // Desired upright angle (in degrees)
double currentAngle = 0;
double angularVelocity = 0;
double error = 0, previousError = 0, integral = 0;
double controlSignal = 0;

// Complementary Filter Variables
double accAngle = 0;      // Angle from accelerometer
double gyroRate = 0;      // Angular velocity from gyroscope
double dt = 0.01;         // Time step for complementary filter (in seconds)
double alpha = 0.995;     // Complementary filter weight

// Time Variables
unsigned long currentTime, previousTime;
double deltaTime;

// Friction Offset (adjust based on floor conditions)
int frictionOffset = 30;

/**
 * Function Name: setup
 * Input: None
 * Output: None
 * Logic: Initializes MPU6050, motor pins, and serial communication.
 * Example Call: setup();
 */
void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Initialize MPU6050
    if (mpu.begin() != 0) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    mpu.calcOffsets(); // Calibrate MPU6050
    Serial.println("MPU6050 initialized!");

    // Initialize motor pins
    pinMode(motor1A, OUTPUT);
    pinMode(motor1B, OUTPUT);
    pinMode(enable1, OUTPUT);
    pinMode(motor2A, OUTPUT);
    pinMode(motor2B, OUTPUT);
    pinMode(enable2, OUTPUT);

    Serial.println("Balancing bot ready!");
    previousTime = millis(); // Initialize time for complementary filter and PID
}

/**
 * Function Name: loop
 * Input: None
 * Output: None
 * Logic: Runs the main control loop including time calculations, sensor updates, PID computation, and motor control.
 * Example Call: loop();
 */
void loop() {
    // Time calculations
    currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0; // Time in seconds
    previousTime = currentTime;

    // Update MPU6050 readings
    mpu.update();

    // Get raw data from MPU6050
    accAngle = mpu.getAngleY();       // Angle from accelerometer (degrees)
    gyroRate = mpu.getGyroY();        // Angular velocity from gyroscope (degrees/sec)

    // Apply complementary filter
    currentAngle = alpha * (currentAngle + gyroRate * dt) + (1 - alpha) * accAngle;

    // PID Control
    error = targetAngle - currentAngle;
    integral += error * dt;

    // Prevent integral wind-up
    integral = constrain(integral, -50, 50);

    double derivative = (error - previousError) / dt;
    previousError = error;

    // Compute control signal
    controlSignal = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Add friction offset
    if (controlSignal > 0) {
        controlSignal += frictionOffset;
    } else if (controlSignal < 0) {
        controlSignal -= frictionOffset;
    }

    // Map control signal to motor PWM values
    int pwmValue = constrain(abs(controlSignal), 0, 255);

    // Motor control based on control signal
    if (controlSignal > 0) {
        // Move forward
        analogWrite(enable1, pwmValue);
        analogWrite(enable2, pwmValue);
        digitalWrite(motor1A, HIGH);
        digitalWrite(motor1B, LOW);
        digitalWrite(motor2A, HIGH);
        digitalWrite(motor2B, LOW);
    } else if (controlSignal < 0) {
        // Move backward
        analogWrite(enable1, pwmValue);
        analogWrite(enable2, pwmValue);
        digitalWrite(motor1A, LOW);
        digitalWrite(motor1B, HIGH);
        digitalWrite(motor2A, LOW);
        digitalWrite(motor2B, HIGH);
    } else {
        // Stop motors
        analogWrite(enable1, 0);
        analogWrite(enable2, 0);
        digitalWrite(motor1A, LOW);
        digitalWrite(motor1B, LOW);
        digitalWrite(motor2A, LOW);
        digitalWrite(motor2B, LOW);
    }

    delay(5); // Small delay for smooth loop execution
}
