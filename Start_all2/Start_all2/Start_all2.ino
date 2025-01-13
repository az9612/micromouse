#include "IMU.h"
#include <Arduino.h>

class PIDController {
private:
    // PID gains
    float Kp, Ki, Kd;
    
    // Internal variables for calculations
    float setpoint;        // Desired value
    float processVariable; // Current value
    float error, prevError;
    float integral, derivative;
    float output;
    
    // Timing for dt calculation
    unsigned long prevTime;

    // Output limits
    float outputMin, outputMax;

public:
    // Constructor
    PIDController(float Kp, float Ki, float Kd, float outputMin = 0, float outputMax = 255)
        : Kp(Kp), Ki(Ki), Kd(Kd), setpoint(0), processVariable(0),
          error(0), prevError(0), integral(0), derivative(0), output(0),
          outputMin(outputMin), outputMax(outputMax), prevTime(0) {}

    // Set the desired setpoint
    void setSetpoint(float sp) {
        setpoint = sp;
    }

    // Set output limits
    void setOutputLimits(float min, float max) {
        outputMin = min;
        outputMax = max;
    }

    // Update the PID controller and calculate the output
    float update(float processVariable, float sp) {
        setpoint = sp;
        this->processVariable = processVariable;
        error = setpoint - processVariable;

        // Calculate time step (dt)
        unsigned long currentTime = millis();
        float dt = (currentTime - prevTime) / 1000.0; // Convert ms to seconds
        prevTime = currentTime;

        // Compute PID terms
        float proportional = Kp * error;
        integral += Ki * error * dt;
        derivative = Kd * (error - prevError) / dt;

        // Calculate total output
        output = proportional + integral + derivative;

        // Constrain output to limits
        if (output > outputMax) {
            output = outputMax;
            // Prevent integral windup
            integral -= Ki * error * dt;
        }
        if (output < outputMin) {
            output = outputMin;
            // Prevent integral windup
            integral -= Ki * error * dt;
        }

        // Save the current error for the next update
        prevError = error;

        return output;
    }
};

// Kalman Filter Structure
struct KalmanFilter {
  float Q_angle;    // Process noise covariance for angle
  float Q_bias;     // Process noise covariance for bias
  float R_measure;  // Measurement noise covariance

  float angle;      // The angle calculated by the Kalman filter
  float bias;       // The gyroscope bias calculated by the filter
  float P[2][2];    // Error covariance matrix

  KalmanFilter() {
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f;
    bias = 0.0f;

    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
  }

  float update(float newAngle, float newRate, float dt) {
    // Predict step
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update step
    float S = P[0][0] + R_measure; // Innovation covariance
    float K[2];                    // Kalman Gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;    // Innovation
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }
};

//byte buttonPin = 21; 
bool blinkFlag = false;

int8_t state = 0;
double currVel;


// Time variables
unsigned long prevTime;
float dt;
unsigned long revTime;

// Define PID gains
float Kp = 2.0;
float Ki = 1.0;
float Kd = 0.5;

// Initialize the PID controller
PIDController pid(Kp, Ki, Kd, -255, 255);


// Instantiate Kalman Filters for pitch, roll, and yaw
KalmanFilter kalmanYaw;

// IMU data
float accX, accY, accZ;
float gyrX, gyrY, gyrZ;
float magX, magY, magZ;
double velX, velY;
double yawMagnetometer;

// Vehicle control
volatile int8_t i = 0;
double setSpeed = 1;


// Infrared sensors
uint8_t distFront = 4000;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  IMU_setup();
  
pinMode(25, OUTPUT);     // IR Emitter 1 Left
pinMode(27, INPUT);      // IR Transmitter 1
pinMode(32, OUTPUT);     // IR Emitter 2 Frontleft
pinMode(39, INPUT);      // IR Transmitter 2
pinMode(16, OUTPUT);     // IR Emitter 3 Front
pinMode(36, INPUT);      // IR Transmitter 3
pinMode(16, OUTPUT);     // IR Emitter 4 Frontright
pinMode(14, INPUT);      // IR Transmitter 4
pinMode(15, OUTPUT);     // IR Emitter 5 Right
pinMode(34, INPUT);     // IR Transmitter 5
pinMode(13, OUTPUT);    // Motor in1
pinMode(14, OUTPUT);    // Motor in2
// pinMode(buttonPin, INPUT_PULLDOWN);
// attachInterrupt(digitalPinToInterrupt(buttonPin), blink, RISING);
// prevTime = millis();
// revTime = prevTime;
}

void loop() {
  // put your main code here, to run repeatedly:
  //state = stateMachine();
      
      readIMU(accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ);
      //myICM.getAGMT();
      //Serial.println(String(accX));

      digitalWrite(25, HIGH);
      delay(1000);
      digitalWrite(25, LOW);
      delay(1000);
      
    
  //Interrupt
  
  // if (blinkFlag == true){
  //   digitalWrite(6, HIGH);
  //   delay(500);
  //   digitalWrite(6, LOW);
  //   delay(500);
  // }
  delay(1000);
}

/*
int stateMachine()
{
  switch(state)
  {
    case 0:     //Calibrate IR and Position
    return 1;
    case 1:     //Vehicle Speed
    {
      float controlSignal_L = pid.update(currVel_L, setSpeedL);
      float controlSignal_R = pid.update(currVel_R, setSpeedR);
      if (controlSignal_L >= 0 && controlSignal_R >= 0)
      {
      driveForward(controlSignal_L, controlSignal_R);
      }
      else if (controlSignal_L < 0 && controlSignal_R < 0)
      {
      driveBackward(abs(controlSignal_L), abs(controlSignal_R));
      }
      else
      {
        driveAngle(controlSignal_L, controlSignal_R, angle);
      }
      return 2;
    }
    case 2:    // All Distances(IR) --> setSpeed from distance to front 
    {
      uint8_t distLeft, distFrontLeft, distFrontRight, distRight;
      readIR(distLeft, distFrontLeft, distFrontRight, distRight);
      if(distLeft<200)
      {
        setSpeedR -= 0.5; 
      }
      else if(distFrontLeft<200)
      {
        setSpeedR -= 0.8; 
      }
      else setSpeedR = setSpeed;
      if(distFrontRight<200)
      {
        setSpeedL -= 0.8;
      }
      else if(distRight<200)
      {
        setSpeedL -= 0.5;
      }
      else setSpeedL = setSpeed;
      return 3;
    }
    case 3:     // Flood-fill
    {
      return 4;
    }
    case 4:       // Orientation
    {
      readIMU(accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ);
      yawMagnetometer = atan2(magY, magX) * 180 / PI;
      if (yawMagnetometer < 0) yawMagnetometer += 360; // Ensure 0-360 range
      // Apply Kalman filter to fuse magnetometer and gyroscope data
      unsigned long currentTime = millis();
      dt = (currentTime - prevTime) / 1000.0;
      prevTime = currentTime;
      float filteredYaw = kalmanYaw.update(yawMagnetometer, gyrZ, dt);
      return 1;
    }
    default:
    return 0;
  }
}

void readIR(uint8_t& distLeft, uint8_t& distFrontLeft, uint8_t& distFrontRight, uint8_t& distRight)
{
  digitalWrite(1, HIGH);
  delayMicroseconds(200);
  distLeft = analogRead(2);
  digitalWrite(1, LOW);
  digitalWrite(3, HIGH);
  delayMicroseconds(200);
  distFrontLeft = analogRead(4);
  digitalWrite(3, LOW);
  digitalWrite(5, HIGH);
  delayMicroseconds(200);
  distFront = analogRead(6);
  digitalWrite(5, LOW);
  digitalWrite(7, HIGH);
  delayMicroseconds(200);
  distFrontRight = analogRead(8);
  digitalWrite(7, LOW);
  digitalWrite(9, HIGH);
  delayMicroseconds(200);
  distRight = analogRead(10);
  digitalWrite(9, LOW);
}

void driveForward(float vel1, float vel2)
{
  analogWrite(13, 0);
  delayMicroseconds(200);
  analogWrite(14, vel);
}

void driveBackward(float vel1, float vel2)
{
  analogWrite(14, 0);
  delayMicroseconds(200);
  analogWrite(13, vel);
}


void cntRPM()
{
  i++;
  if(i == 50)
  {
    unsigned long delta_Time = millis() - revTime; 
    revTime = millis();
    float omega = 1/((float)delta_Time/1000);
    currVel = omega * 0.01;
    i = 0;
  }
}
*/
void blink()
{
  if(blinkFlag == false) blinkFlag = true;
  else blinkFlag = false; digitalWrite(6, LOW);
}
