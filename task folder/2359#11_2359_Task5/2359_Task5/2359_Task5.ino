#include <Wire.h>
#include <MPU6050_light.h>
#include <SoftwareSerial.h>
#include <Servo.h> // Include Servo library

// Bluetooth pins
SoftwareSerial bluetooth(A1, A0); // RX, TX

// MPU6050 setup
MPU6050 mpu(Wire);

// Motor pins
int motor1A = A2, motor1B = A3, enable1 = 6; // Motor 1 (left)
int motor2A = 4, motor2B = 9, enable2 = 5;  // Motor 2 (right)

const int encoderLeftA = 2; // Left encoder channel A
const int encoderLeftB = 7; // Left encoder channel B
const int encoderRightA = 3; // Right encoder channel A
const int encoderRightB = 8; // Right encoder channel B



// PID Constants
double Kp = 25.5;  // Proportional gain
double Ki =170; // Integral gain
double Kd = 1;   // Derivative gain

// Serial PID time constants
double Ti = Kp / Ki; // Integral time constant
double Td = Kd / Kp; // Derivative time constant

// System state
double targetAngle = 0; 
double currentAngle = 0;
double error = 0, previousError = 0, integral = 0;
double controlSignal = 0;

// Complementary Filter Variables
double accAngle = 0;      // Angle from accelerometer
double gyroRate = 0;      // Angular velocity from gyroscope
double dt = 0.001;        // Time step for complementary filter (in seconds)
double alpha = 0.987;      // Complementary filter weight

// Time variables
unsigned long currentTime, previousTime;

// Friction offset (adjust based on floor conditions)
int frictionOffset = 12;
int speed;

// Bluetooth Commands
char command;

// Turning speed control
int turnSpeed = 0;  // Speed adjustment for turning (-255 to 255)

int maxSpeed = 200; // Max PWM for smooth turning


bool change = false;
double turn =0;
bool rotate = false;


// Define the servos for the robotic hand
Servo servoJ;
Servo servoK;

// Initialize servo angles
int angleJ ;
int angleK ;
bool move = false;


// Position PID Constants
double posKp = 0.0009;   // Position proportional gain
double posKi = 0.000004;  // Position integral gain
double posKd = 0.0007;   // Position derivative gain

// Position control variables
double positionTarget = 0;      // Desired position (in encoder counts)
double positionError = 0;
double positionIntegral = 0, positionPreviousError = 0;
double positionControl = 0;



// Speed variables (counts per second)
double leftWheelSpeed = 0;
double rightWheelSpeed = 0;

// Time variables for speed calculation
unsigned long lastSpeedCalcTime = 0;
const int speedCalcInterval = 100; // Interval to calculate speed in milliseconds
const double wheelCircumference = 20.42; // Example: 6.5 cm diameter wheel (π * d)
const int countsPerRevolution = 360;    // Example: Encoder has 360 counts per revolution



// Encoder variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

/**
 * Function Name: setup
 * Input: None
 * Output: None
 * Logic: Initializes all components including MPU6050, motors, and servos.
 * Example Call: setup();
 */
// Interrupt Service Routines for encoders
void leftEncoderISR() {
  if (digitalRead(encoderLeftA) == digitalRead(encoderLeftB)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderISR() {
  if (digitalRead(encoderRightA) == digitalRead(encoderRightB)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}


void calculateSpeed() {
  static long lastLeftEncoderCount = 0;
  static long lastRightEncoderCount = 0;

  // Calculate time interval in seconds
  unsigned long currentTime = millis();
  double elapsedTime = (currentTime - lastSpeedCalcTime) / 1000.0;
  lastSpeedCalcTime = currentTime;

  // Calculate left wheel speed
  long leftDelta = leftEncoderCount - lastLeftEncoderCount;
  leftWheelSpeed = leftDelta / elapsedTime; // Speed in counts per second
  lastLeftEncoderCount = leftEncoderCount;

  // Calculate right wheel speed
  long rightDelta = rightEncoderCount - lastRightEncoderCount;
  rightWheelSpeed = rightDelta / elapsedTime; // Speed in counts per second
  lastRightEncoderCount = rightEncoderCount;
  
  if(rotate){

  }else {
    if(rightWheelSpeed>leftWheelSpeed){
      turnSpeed += 3;
    }else{
      turnSpeed -= 3;
    }
  }
}




void setup() {
  
   // Encoder initialization
  pinMode(encoderLeftA, INPUT);
  pinMode(encoderLeftB, INPUT);
  pinMode(encoderRightA, INPUT);
  pinMode(encoderRightB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, CHANGE);


  Serial.begin(9600);        // For serial monitor
  bluetooth.begin(9600);     // For HC-05 (default baud rate is 9600)

  // Initialize MPU6050
  Wire.begin();
  if (mpu.begin() != 0) {
    while (1);
  }
  // Set Gyroscope Offsets
    mpu.setGyroOffsets(-1.72, -0.45, 0.76);  // Set offsets for X, Y, Z axes

    // Set Accelerometer Offsets
    mpu.setAccOffsets(0.03, -0.02, 0.06);    // Set offsets for X, Y, Z axes

  // Initialize motors
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(enable2, OUTPUT);

  servoJ.attach(10);      // Attach servo "J" to pin 9
  servoK.attach(11);     // Attach servo "K" to pin 10
  servoJ.write(87);       // Initialize servo "J" at 0 degrees
  servoK.write(0);       // Initialize servo "K" at 0 degrees

  previousTime = millis(); // Initialize time for complementary filter and PID
}

/**
 * Function Name: loop
 * Input: None
 * Output: None
 * Logic: Main loop for reading sensor data, handling commands, and controlling motors.
 * Example Call: loop();
 */
void loop() {
  while (bluetooth.available()) {
    command = bluetooth.read();
    if (command != '\r' && command != '\n' && command != ' ') {
      Serial.println(command);
      handleCommand(command);
    }

  }

  if (millis() - lastSpeedCalcTime >= speedCalcInterval) {
    calculateSpeed();
  }

  // Calculate current position (average of both encoders)
  long currentPosition = (rightEncoderCount + leftEncoderCount) / 2;

  // Position PID Control
  positionError = positionTarget - currentPosition;

  positionIntegral += positionError * dt;
  double positionDerivative = (positionError - positionPreviousError) / dt;
  positionControl = posKp * positionError + posKi * positionIntegral + posKd * positionDerivative;
  positionPreviousError = positionError;

  // Adjust target angle based on position control
  targetAngle = positionControl;



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

  // PID Control (Serial Implementation)
  error = targetAngle - currentAngle;

  // Proportional term
  double proportional = Kp * error;

  // Integral term
  integral += error * dt;
  double integralTerm = (Kp / Ti) * integral;

  // Derivative term
  double derivative = (error - previousError) / dt;
  double derivativeTerm = (Kp * Td) * derivative;



  if (move){
    positionTarget -= speed;
  }


  // Compute control signal
  controlSignal = proportional + integralTerm + derivativeTerm;

  previousError = error;

  // Add friction offset
  if (controlSignal > 0) controlSignal += frictionOffset;
  else if (controlSignal < 0) controlSignal -= frictionOffset;

  // Map control signal to motor PWM values
  int pwmValue = constrain(abs(controlSignal), 0, 255);

  

  // Combine balancing and speed control signals
  int leftMotorSpeed = constrain(abs(controlSignal - turnSpeed), 0, 255);
  int rightMotorSpeed = constrain(abs(controlSignal + turnSpeed), 0, 255);

  // Motor control
  controlMotors(controlSignal, leftMotorSpeed, rightMotorSpeed);

  delay(5); // Small delay for smooth loop execution
}

/**
 * Function Name: handleCommand
 * Input: cmd - Bluetooth command character.
 * Output: None
 * Logic: Handles Bluetooth commands for movement and speed adjustments.
 * Example Call: handleCommand('F');
 */
// Handles incoming Bluetooth commands
void handleCommand(char cmd) {
  switch (cmd) {


    case 'F': // Move forward
      speed = 10.5;
      move = true;
      break;
    case 'f': // Move forward
      speed = 9.5;
      move = true;
      break;

      case 'FR': // Move forward
      speed = 9;
      move = true;
      turnSpeed = 30;
      break;
      case 'FL': // Move forward
      speed = 9;
      move = true;
      turnSpeed = -30;
      break;
      
    case 'B': // Move backward
      speed = -10.5;
      move = true;
      break;

      case 'BL': // Move forward
      speed = -9;
      move = true;
      turnSpeed = 30;
      break;
      case 'BR': // Move forward
      speed = -9;
      move = true;
      turnSpeed = -30;
      break;


    case 'X':
      speed = 14.5;
      move = true;
      break;
    case 'Y':
      speed = -14.5;
      move = true;
      break;
     
    case 'M':
      tone(12,1000);
      break;
    case 'm':
      noTone(12);
      break;

    case 'L': 
        turnSpeed = -35;
        rotate = true;
      break;
    case 'R':
        turnSpeed = 35;
        rotate = true;
      break;


      case 'U': // Stop
      servoK.write(0);
      break;

      case 'D': // Stop
      servoK.write(50);
      break;

      case 'O': // Stop
      servoJ.write(65);
      break;

      case 'C': // Stop
      servoJ.write(115);
      break;

      case 'S':
      move = false;
      break;


      case 'A':
        turnSpeed = 0;
        rotate = false;
      break;
      
      case 'Z': // Stop
        servoJ.write(87);
      break;



      case 'SA':
        move = false;
        turnSpeed = 0;
        rotate = false;
      break;
  }
}


// Controls motor direction and speed
void controlMotors(double controlSignal, int leftSpeed, int rightSpeed) {
  if (controlSignal > 0) { // Move forward
    digitalWrite(motor1A, HIGH);
    digitalWrite(motor1B, LOW);
    digitalWrite(motor2A, HIGH);
    digitalWrite(motor2B, LOW);
  } else { // Move backward
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, HIGH);
    digitalWrite(motor2A, LOW);
    digitalWrite(motor2B, HIGH);
  }

  analogWrite(enable1, leftSpeed);
  analogWrite(enable2, rightSpeed);
}