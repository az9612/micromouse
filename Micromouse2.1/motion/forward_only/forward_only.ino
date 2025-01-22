#include <AutoPID.h>

// Encoder pins for left and right motors
#define ENC_PIN_LEFT 26
#define ENC_PIN_RIGHT 35

// Motor control pins for left motor
#define mleft_1 4
#define mleft_2 0

#define button 12

// Motor control pins for right motor
#define mright_1 17
#define mright_2 33

// PWM parameters
#define PWM_FREQ 1000     // PWM frequency
#define PWM_PRECISION 12  // PWM resolution in bits
#define PWM_MAX 4095      // Maximum PWM value
#define CHN_LEFT1 0       // PWM channel for left motor
#define CHN_RIGHT1 1      // PWM channel for right motor
#define CHN_LEFT2 3
#define CHN_RIGHT2 4
#define OUTPUT_MIN 0
#define OUTPUT_MAX 4095
#define KP 1.196
#define KI 0.6305
#define KD 0.4436
#define KPr 1.688
#define KIr 0.6222
#define KDr 0.2539

// Variables for encoder counts and speed
volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;
double currentSpeedLeft = 0;
double currentSpeedRight = 0;
const unsigned long sampleTime = 100;  // PID loop interval in milliseconds
unsigned long lastTime = 0;
bool loopActive = false;

// Target speed for both motors (pulses per second)
double targetSpeed = 800;
//double targetSpeedR = 1.05 * targetSpeedL;
double outputValL, outputValR;
AutoPID myPIDL(&currentSpeedLeft, &targetSpeed, &outputValL, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID myPIDR(&currentSpeedRight, &targetSpeed, &outputValR, OUTPUT_MIN, OUTPUT_MAX, KPr, KIr, KDr);

void setup() {
  // Configure encoder pins
  pinMode(ENC_PIN_LEFT, INPUT_PULLUP);
  pinMode(ENC_PIN_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_LEFT), countPulsesLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_RIGHT), countPulsesRight, RISING);

  // Configure motor control pins
  pinMode(mleft_1, OUTPUT);
  pinMode(mright_1, OUTPUT);
  pinMode(mleft_2, OUTPUT);
  pinMode(mright_2, OUTPUT);

  pinMode(button, INPUT);

  // Set motor direction to forward (HIGH)
  // digitalWrite(MOTOR_DIR_LEFT, 10);
  // digitalWrite(MOTOR_DIR_RIGHT, 10);

  // Attach PWM channels to pins
  ledcAttachChannel(mleft_1, PWM_FREQ, PWM_PRECISION, CHN_LEFT1);
  ledcAttachChannel(mright_1, PWM_FREQ, PWM_PRECISION, CHN_RIGHT1);
  ledcAttachChannel(mleft_2, PWM_FREQ, PWM_PRECISION, CHN_LEFT2);
  ledcAttachChannel(mright_2, PWM_FREQ, PWM_PRECISION, CHN_RIGHT2);

  myPIDL.setTimeStep(100);
  myPIDR.setTimeStep(100);
  // Start serial for debugging
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (digitalRead(button == HIGH)) {
    delay(20);
    if(digitalRead(button == HIGH)){
      loopActive = true;
    }    
  }
    while(loopActive){
    moveForward();
    }
  //moveForward();

  // Serial.print("Left Speed: ");
  // Serial.print(currentSpeedLeft);
  // Serial.print(" | Right Speed: ");
  // Serial.print(currentSpeedRight);
  // Serial.print(" | Left PWM Output: ");
  // Serial.print(outputValL);
  // Serial.print(" | Right PWM Output: ");
  // Serial.println(outputValR);
}

// Interrupt Service Routine (ISR) for counting left encoder pulses
void countPulsesLeft() {
  pulseCountLeft++;
}

// Interrupt Service Routine (ISR) for counting right encoder pulses
void countPulsesRight() {
  pulseCountRight++;
}

// void turnRight(int pin1, int pin2, double speed) {
//   while (pulseCountRight < calculated encoder Value) {
//     // Ensure speed is within valid range
//     speed = constrain(speed, 500, PWM_MAX);
//     // Convert double to uint32_t
//     uint32_t dutyCycle = static_cast<uint32_t>(speed);
//     ledcWrite(pin1, dutyCycle);  // Set PWM
//     digitalWrite(pin2, LOW);
//   }
// }

// void turnLeft(int pin1, int pin2, double speed) {
//   while (pulseCountRight < calculated encoder Value) {
//     // Ensure speed is within valid range
//     speed = constrain(speed, 500, PWM_MAX);
//     // Convert double to uint32_t
//     uint32_t dutyCycle = static_cast<uint32_t>(speed);
//     ledcWrite(pin1, dutyCycle);  // Set PWM
//     digitalWrite(pin2, LOW);
//   }
// }

// void stopMove() {
//   setMotorSpeed(mleft_1, mleft_2, 0);
//   setMotorSpeed(mright_1, mright_2, 0);
// }

void moveForward() {
  getEncSpeed();
  myPIDL.run();
  myPIDR.run();
  setMotorSpeed(mleft_1, mleft_2, outputValL);
  setMotorSpeed(mright_1, mright_2, outputValR);
}

void getEncSpeed() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;

  if (elapsedTime >= sampleTime) {
    lastTime = currentTime;
    //double elapsedTimeSec = elapsedTime / 1000.0;  // Convert milliseconds to seconds

    // Calculate current speed for left motor
    noInterrupts();
    long pulsesLeft = pulseCountLeft;
    pulseCountLeft = 0;
    long pulsesRight = pulseCountRight;

    pulseCountRight = 0;
    interrupts();
    currentSpeedLeft = (double)pulsesLeft / (sampleTime / 1000.0);
    currentSpeedRight = (double)pulsesRight / (sampleTime / 1000.0);
  }
}

void setMotorSpeed(int pin1, int pin2, double speed) {
  // Ensure speed is within valid range
  speed = constrain(speed, 700, PWM_MAX);
  // Convert double to uint32_t
  uint32_t dutyCycle = static_cast<uint32_t>(speed);

  ledcWrite(pin1, dutyCycle);  // Set PWM
  digitalWrite(pin2, LOW);
}
