// Encoder pins for left and right motors
#define ENC_PIN_LEFT 2
#define ENC_PIN_RIGHT 3

// Motor control pins for left motor
#define MOTOR_PWM_LEFT 4
#define MOTOR_DIR_LEFT 0

// Motor control pins for right motor
#define MOTOR_PWM_RIGHT 10
#define MOTOR_DIR_RIGHT 8

// Variables for encoder counts and speed
volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;
double currentSpeedLeft = 0;
double currentSpeedRight = 0;
const unsigned long sampleTime = 100; // PID loop interval in milliseconds
unsigned long lastTime = 0;

// Target speed for both motors (pulses per second)
const double targetSpeed = 100;

// PID variables for left motor
double leftError = 0, leftIntegral = 0, leftDerivative = 0, leftLastError = 0, leftPidOutput = 0;

// PID variables for right motor
double rightError = 0, rightIntegral = 0, rightDerivative = 0, rightLastError = 0, rightPidOutput = 0;

// PID constants (same for both motors)
const double Kp = 1.0;  // Proportional gain
const double Ki = 0.5;  // Integral gain
const double Kd = 0.1;  // Derivative gain

void setup() {
  // Configure encoder pins
  pinMode(ENC_PIN_LEFT, INPUT_PULLUP);
  pinMode(ENC_PIN_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_LEFT), countPulsesLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_RIGHT), countPulsesRight, RISING);

  // Configure motor control pins
  pinMode(MOTOR_PWM_LEFT, OUTPUT);
  pinMode(MOTOR_DIR_LEFT, OUTPUT);
  pinMode(MOTOR_PWM_RIGHT, OUTPUT);
  pinMode(MOTOR_DIR_RIGHT, OUTPUT);

  // Start serial for debugging
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();

  // Update PID at regular intervals
  if (currentTime - lastTime >= sampleTime) {
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
    leftDerivative = (leftError - leftLastError) / (sampleTime / 1000.0);
    leftPidOutput = Kp * leftError + Ki * leftIntegral + Kd * leftDerivative;
    leftPidOutput = constrain(leftPidOutput, 0, 255);
    leftLastError = leftError;

    // PID calculations for right motor
    rightError = targetSpeed - currentSpeedRight;
    rightIntegral += rightError * (sampleTime / 1000.0);
    rightDerivative = (rightError - rightLastError) / (sampleTime / 1000.0);
    rightPidOutput = Kp * rightError + Ki * rightIntegral + Kd * rightDerivative;
    rightPidOutput = constrain(rightPidOutput, 0, 255);
    rightLastError = rightError;

    // Set motor speeds
    setMotorSpeed(MOTOR_PWM_LEFT, MOTOR_DIR_LEFT, leftPidOutput);
    setMotorSpeed(MOTOR_PWM_RIGHT, MOTOR_DIR_RIGHT, rightPidOutput);

    // Debugging output
    Serial.print("Left Speed: ");
    Serial.print(currentSpeedLeft);
    Serial.print(" | Right Speed: ");
    Serial.print(currentSpeedRight);
    Serial.println();
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
void setMotorSpeed(int pwmPin, int dirPin, double speed) {
  digitalWrite(dirPin, HIGH); // Forward direction
  analogWrite(pwmPin, constrain(speed, 0, 255)); // Set PWM
}
