// Encoder pins for left and right motors
#define ENC_PIN_LEFT 26
#define ENC_PIN_RIGHT 35

// Motor control pins for left motor
#define MOTOR_PWM_LEFT 0
#define MOTOR_DIR_LEFT 4

// Motor control pins for right motor
#define MOTOR_PWM_RIGHT 17
#define MOTOR_DIR_RIGHT 33

// PWM parameters
#define PWM_FREQ 1000     // PWM frequency
#define PWM_PRECISION 8   // PWM resolution in bits
#define PWM_MAX 255       // Maximum PWM value
#define CHN_LEFT 0        // PWM channel for left motor
#define CHN_RIGHT 1       // PWM channel for right motor

// Variables for encoder counts and speed
volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;
double currentSpeedLeft = 0;
double currentSpeedRight = 0;
const unsigned long sampleTime = 500; // PID loop interval in milliseconds
unsigned long lastTime = 0;

// Target speed for both motors (pulses per second)
const double targetSpeed = 50;

// PID variables for left motor
double leftError = 0, leftIntegral = 0, leftDerivative = 0, leftLastError = 0, leftPidOutput = 0;

// PID variables for right motor
double rightError = 0, rightIntegral = 0, rightDerivative = 0, rightLastError = 0, rightPidOutput = 0;

// PID constants (same for both motors)
const double Kp = 2.0;  // Proportional gain
const double Ki = 0.0;  // Integral gain
const double Kd = 0.0;  // Derivative gain

void setup() {
  // Configure encoder pins
  pinMode(ENC_PIN_LEFT, INPUT_PULLUP);
  pinMode(ENC_PIN_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_LEFT), countPulsesLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_RIGHT), countPulsesRight, RISING);

  // Configure motor control pins
  pinMode(MOTOR_DIR_LEFT, OUTPUT);
  pinMode(MOTOR_DIR_RIGHT, OUTPUT);

  // Attach PWM channels
  ledcAttachChannel(MOTOR_PWM_LEFT, PWM_FREQ, PWM_PRECISION, CHN_LEFT);
  ledcAttachChannel(MOTOR_PWM_RIGHT, PWM_FREQ, PWM_PRECISION, CHN_RIGHT);

  // Start serial for debugging
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;

  // Update PID at regular intervals
  if (elapsedTime >= sampleTime) {
    lastTime = currentTime;

    // Calculate current speed for left motor
    noInterrupts();
    long pulsesLeft = pulseCountLeft;
    pulseCountLeft = 0;
    interrupts();
    currentSpeedLeft = (double)pulsesLeft / (sampleTime / 1000.0);

    // Calculate current speed for right motor
    noInterrupts();
    long pulsesRight = pulseCountRight;
    pulseCountRight = 0;
    interrupts();
    currentSpeedRight = (double)pulsesRight / (sampleTime / 1000.0);

    // PID calculations for left motor
    leftError = targetSpeed - currentSpeedLeft;
    leftIntegral += leftError * (sampleTime / 1000.0);
    leftIntegral = constrain(leftIntegral, -50, 50); // Prevent windup
    leftDerivative = (leftError - leftLastError) / (sampleTime / 1000.0);
    leftPidOutput = Kp * leftError + Ki * leftIntegral + Kd * leftDerivative;
    leftPidOutput = constrain(leftPidOutput, -PWM_MAX, PWM_MAX);
    leftLastError = leftError;

    // PID calculations for right motor
    rightError = targetSpeed - currentSpeedRight;
    rightIntegral += rightError * (sampleTime / 1000.0);
    rightIntegral = constrain(rightIntegral, -50, 50); // Prevent windup
    rightDerivative = (rightError - rightLastError) / (sampleTime / 1000.0);
    rightPidOutput = Kp * rightError + Ki * rightIntegral + Kd * rightDerivative;
    rightPidOutput = constrain(rightPidOutput, -PWM_MAX, PWM_MAX);
    rightLastError = rightError;

    // Set motor speeds
    setMotorSpeed(CHN_LEFT, MOTOR_DIR_LEFT, leftPidOutput);
    setMotorSpeed(CHN_RIGHT, MOTOR_DIR_RIGHT, rightPidOutput);

    // Debugging output
    Serial.print("Left Speed: ");
    Serial.print(currentSpeedLeft);
    Serial.print(" | Right Speed: ");
    Serial.print(currentSpeedRight);
    Serial.print(" | Left PID Output: ");
    Serial.print(leftPidOutput);
    Serial.print(" | Right PID Output: ");
    Serial.println(rightPidOutput);
  }
}

// Interrupt Service Routine (ISR) for counting left encoder pulses
void countPulsesLeft() {
  pulseCountLeft++;
}

// Interrupt Service Routine (ISR) for counting right encoder pulses
void countPulsesRight() {
  pulseCountRight++;
}

// Function to control motor speed and direction
void setMotorSpeed(int channel, int dirPin, double speed) {
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH); // Forward direction
  } else {
    digitalWrite(dirPin, LOW); // Reverse direction
    speed = -speed; // Make speed positive for PWM
  }
  ledcWrite(channel, constrain(speed, 0, PWM_MAX)); // Set PWM
}
