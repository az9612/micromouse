class PIDController {
private:
    // PID gains
    float Kp, Ki, Kd;
    
    // Internal variables for calculations
    float setpoint;        // Desired value
    float processVariable; // Current value
    float error, prevError;
    float proportional, integral, derivative;
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
        proportional = Kp * error;
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

//#include "ICM_20948.h"
#include <Adafruit_NeoPixel.h>
#include <math.h>

// Define the LED strip configuration
#define LED_PIN    13  // Pin connected to the data input of the LED strip
#define NUM_LEDS   3 // Number of LEDs in the strip

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define cs 5 


#define I2C_ADDRESS 0x3C

// Create a NeoPixel object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

//ICM_20948_I2C myICM;

byte IR_Emit_Left = 25;
byte IR_Trans_Left = 27;

byte IR_Emit_FrontLeft = 2;
byte IR_Trans_FrontLeft = 14;

byte IR_Emit_Front = 15;
byte IR_Trans_Front = 34;

byte IR_Emit_FrontRight = 16;
byte IR_Trans_FrontRight = 36;

byte IR_Emit_Right = 32;
byte IR_Trans_Right = 39;

byte buttonPin = 12;

byte motorL_in1 = 4;
byte motorL_in2 = 0;
byte encPinL = 26;

byte motorR_in1 = 17;
byte motorR_in2 = 33;
byte encPinR = 35;

unsigned long last_Time;
unsigned long start_Time;
volatile uint32_t i_L;
volatile uint32_t i_R;
int64_t cnt00_L;
int64_t cnt00_R;
int64_t cnt01_L;
int64_t cnt01_R;
int64_t n = pow(2,32)-1;

bool blinker = false;

uint16_t  distFrontLeft, distFrontRight, distL, distR,distF;
double distRight, distLeft, distFront;

double angle;
bool turnright;
bool turnleft;

// Time variables
unsigned long prevTime;
float dt;
unsigned long revTime;

PIDController speedControllerL{1.0, 0.0, 0.5, 40, 255}; // {Kp, Ki, Kd, outputMin, OutputMax}
PIDController speedControllerR{1.0, 0.0, 0.5, 40, 255};


PIDController wallLeftController{3.0, 3.0, 0.05, 0.0, 1.0};
PIDController wallRightController{3.0, 3.0, 0.05, 0.0, 1.0};

PIDController wallFrontController{5.0, 8.0, 0.1, 1.0, 0.0};

void setup() 
{
  // put your setup code here, to run once:
  pinMode(IR_Emit_Left, OUTPUT);     // IR Emitter 1 Left
  pinMode(IR_Trans_Left, INPUT);      // IR Transmitter 1

  pinMode(IR_Emit_FrontLeft, OUTPUT);     // IR Emitter 2 Frontleft
  pinMode(IR_Trans_FrontLeft, INPUT);      // IR Transmitter 2

  pinMode(IR_Emit_Front, OUTPUT);     // IR Emitter 3 Front
  pinMode(IR_Trans_Front, INPUT);      // IR Transmitter 3

  pinMode(IR_Emit_FrontRight, OUTPUT);     // IR Emitter 4 Frontright
  pinMode(IR_Trans_FrontRight, INPUT);      // IR Transmitter 4

  pinMode(IR_Emit_Right, OUTPUT);     // IR Emitter 5 Right
  pinMode(IR_Trans_Right, INPUT);     // IR Transmitter 5

  pinMode(motorL_in1, OUTPUT);    // Motor in1
  pinMode(motorL_in2, OUTPUT);    // Motor in2

  pinMode(motorR_in1, OUTPUT);    // Motor in1
  pinMode(motorR_in2, OUTPUT);    // Motor in2

  pinMode(buttonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin), blink, RISING);

  pinMode(encPinL, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinL), cntRPM_L, RISING);

  pinMode(encPinR, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinR), cntRPM_R, RISING);

  strip.begin(); // Initialize the LED strip
  strip.show();  // Turn off all LEDs

  prevTime = millis();
  revTime = prevTime;
  start_Time = millis();
  Serial.begin(115200);
  //IMU_setup();
  /*
  if (!SD.begin(cs)) {
    Serial.printf("Card Mount Failed \n");
    return;
  } else {
    Serial.printf("Card accepted");
  }
  writeFile(SD, "/test.txt", "Start1 \n");
  */
}
/*
void IMU_setup()
{
  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while (!initialized)
  {

    myICM.begin(Wire, 1);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  Serial.println(F("Device connected!"));

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("Software Reset returned: "));
    Serial.println(myICM.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setSampleMode returned: "));
    Serial.println(myICM.statusString());
  }
  
  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps500; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setFullScale returned: "));
    Serial.println(myICM.statusString());
  }
  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setDLPcfg returned: "));
    Serial.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
  Serial.print(F("Enable DLPF for Accelerometer returned: "));
  Serial.println(myICM.statusString(accDLPEnableStat));
  Serial.print(F("Enable DLPF for Gyroscope returned: "));
  Serial.println(myICM.statusString(gyrDLPEnableStat));

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("startupMagnetometer returned: "));
    Serial.println(myICM.statusString());
  }

  Serial.println();
  Serial.println(F("Configuration complete!"));
}

void readIMU(float& x_acc, float& y_acc, float& z_acc, float& x_gyr, float& y_gyr, float& z_gyr, float& x_mag, float& y_mag, float& z_mag)
{
  if(myICM.dataReady())
  {
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
   
    x_acc = myICM.accX();
    y_acc = myICM.accY();
    z_acc = myICM.accZ();
  
    x_gyr = myICM.gyrX();
    y_gyr = myICM.gyrY();
    z_gyr = myICM.gyrZ();
    
    x_mag = myICM.magX();
    y_mag = myICM.magY();
    z_mag = myICM.magZ();
    
    //delay(30);
  }
  else
  {
    Serial.println("Waiting for data");
    //delay(500);
  }
}
*/
void readIR(double& distLeft, uint16_t& distFrontLeft, double& distFront, uint16_t& distFrontRight, double& distRight)
{
  digitalWrite(IR_Emit_Left, HIGH);
  delayMicroseconds(100);
  distL = analogRead(IR_Trans_Left);
  digitalWrite(IR_Emit_Left, LOW);

  digitalWrite(IR_Emit_FrontLeft, HIGH);
  delayMicroseconds(100);
  distFrontLeft = analogRead(IR_Trans_FrontLeft);
  digitalWrite(IR_Emit_FrontLeft, LOW);

  digitalWrite(IR_Emit_Front, HIGH);
  delayMicroseconds(100);
  distF = analogRead(IR_Trans_Front);
  digitalWrite(IR_Emit_Front, LOW);

  digitalWrite(IR_Emit_FrontRight, HIGH);
  delayMicroseconds(100);
  distFrontRight = analogRead(IR_Trans_FrontRight);
  digitalWrite(IR_Emit_FrontRight, LOW);

  digitalWrite(IR_Emit_Right, HIGH);
  delayMicroseconds(100);
  distR = analogRead(IR_Trans_Right);
  digitalWrite(IR_Emit_Right, LOW);

  distLeft = pow(distL, 3) * 2.7458e-09 + pow(distL, 2) * -7.9716e-06 + distL * 0.0146 + 12.5175;
  distFront = pow(distF, 5) * 8.9719e-15 + pow(distF, 4) * -5.7233e-11 + pow(distF, 3) * 1.3600e-07 
              + pow(distF, 2) * -1.4151e-04 + distF * 0.0707 + 13.8345;
  distRight = pow(distR, 3) * 1.6985e-09 + pow(distR, 2) * -6.2843e-06 + distR * 0.0124 + 12.3945;
}

void driveBackward(float velL, float velR)
{
  analogWrite(motorL_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in2, velL);
  analogWrite(motorR_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in2, velR);
}

void driveForward(float velL, float velR)
{
  analogWrite(motorL_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in1, velL);
  analogWrite(motorR_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in1, velR);
}

void rotateRight(float velL, float velR)
{
  analogWrite(motorL_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in2, velL);
  analogWrite(motorR_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in1, velR);

  //double gyro = myICM.gyrZ();
  double gyro = 0;

  angle += gyro*0.005;
  if(angle >= 90)
  {
    turnright = true;
    angle = 0;
  }
}

void rotateLeft(float velL, float velR)
{
  analogWrite(motorL_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in1, velL);
  analogWrite(motorR_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in2, velR);

  //double gyro = myICM.gyrZ();
  double gyro = 0;
  angle += gyro*0.005;
  if(angle >= -90)
  {
    turnleft = true;
    angle = 0;
  }
}

void cntRPM_L()
{
  i_L++;
}

void cntRPM_R()
{
  i_R++;
}

void blink()
{
  blinker = true;
  start_Time = millis();
}

void get_AngularVelocities(double& in1, double& in2)
{
  cnt00_L = i_L;
  cnt00_R = i_R;
  /*
  if(cnt01_L > cnt00_L)
  {
    cnt00_L += n;
  }
  cnt01_L = cnt00_L - cnt01_L;
  
  if(cnt01_R > cnt00_R)
  {
    cnt00_R += n;
  }
  */
  cnt01_R = cnt00_R - cnt01_R;
  cnt01_L = cnt00_L - cnt01_L;

  unsigned long timer = millis();
  double dtime = timer - revTime;
  revTime = timer;
  dtime = dtime/1000; 
  double speed_R, speed_L;
  uint8_t ticks_per_rot = 140;
  double zwischenrechnerL = (double)cnt01_L/(double)ticks_per_rot;
  double zwischenrechnerR = (double)cnt01_R/(double)ticks_per_rot;
  speed_L = zwischenrechnerL / dtime;
  speed_R = zwischenrechnerR / dtime;
  cnt01_L = cnt00_L;
  cnt01_R = cnt00_R;
  //Serial.println("SpeedL: " + String(speed_L) + " SpeedR: " + String(speed_R));
  //Serial.println("Zwischen_L: " + String(zwischenrechnerL) + " Zwischen_R: " + String(zwischenrechnerR));
  //Serial.println("Time: " + String(dtime));
  //Serial.println("Ticks_L: " + String(cnt00_L) + " Ticks_R: " + String(cnt00_R));
  //return zwischenrechnerL, zwischenrechnerR;
  in1 = speed_L;
  in2 = speed_R;
}

void get_LinearVelocity(double& speed_L, double& speed_R)
{
  double omega_L, omega_R ;
  get_AngularVelocities(omega_L, omega_R); 
  float r = 0.019; 
  speed_L = 2 * M_PI * r * omega_L; // m/s linear speed
  speed_R = r * 2 * M_PI * omega_R;
  //Serial.println("SpeedL: " + String(speed_L) + " SpeedR: " + String(speed_R));
}

void get_LinearVelocities(double& speed_L, double& speed_R)
{
  double omega_L = speed_L;
  double omega_R = speed_R; 
  float r = 0.019; 
  speed_L = 2*r * M_PI * omega_L; // m/s linear speed
  speed_R = 2*r * M_PI * omega_R;
  
  //Serial.println("omegaL: " + String(omega_L) + " omegaR: " + String(omega_R));
  Serial.println("LinSpeedL: " + String(speed_L) + " LinSpeedR: " + String(speed_R));
}

void scenerioOne() //max possible accel without controller
{
  double ang_speedL, ang_speedR, lin_speedL, lin_speedR, speedL, speedR;
  unsigned long timee = millis();
  driveForward(255, 255);
  get_AngularVelocities(speedL, speedR);
  get_LinearVelocities(speedL, speedR);
  //save in sd
}

void scenerioTwo() //max possible accel with controller
{
  double ang_speedL, ang_speedR, lin_speedL, lin_speedR, speedL, speedR, setSpeed;
  setSpeed = 1.0;
  unsigned long timee = millis();
  
  get_AngularVelocities(speedL, speedR);
  //Serial.println("Angular_L: " + String(speedL));
  ang_speedL = speedL;
  ang_speedR = speedR;
  get_LinearVelocities(speedL, speedR);
  lin_speedL = speedL;
  lin_speedR = speedR;
  float controlSignal_L = speedControllerL.update(speedL, setSpeed);
  float controlSignal_R = speedControllerR.update(speedR, setSpeed);
  Serial.println("Signal_L: " + String(controlSignal_L)+ " Signal_R: " + String(controlSignal_R));
  driveForward((int)controlSignal_L, (int)controlSignal_R);
  //driveForward(40, 40);
  
  //save in sd
  //saveToSD(timee, ang_speedL, ang_speedR, lin_speedL, lin_speedR, controlSignal_L, controlSignal_R);
}

void scenerioThree() //drive in between walls
{
  double ang_speedL, ang_speedR, lin_speedL, lin_speedR, speedL, speedR, setSpeedL, setSpeedR;
  setSpeedL = wallLeftController.update(distLeft, 800); // fixed value for analog wall distance --> maybe declaration with calibration
  setSpeedR = wallLeftController.update(distRight, 800);
  unsigned long timee = millis();
  get_AngularVelocities(speedL, speedR);
  ang_speedL = speedL;
  ang_speedR = speedR;
  get_LinearVelocities(speedL, speedR);
  lin_speedL = speedL;
  lin_speedR = speedR;
  float controlSignal_L = speedControllerL.update(speedL, setSpeedL);
  float controlSignal_R = speedControllerR.update(speedR, setSpeedR);
  driveForward(controlSignal_L, controlSignal_R);

  //save in sd
  //saveToSD(timee, ang_speedL, ang_speedR, lin_speedL, lin_speedR, controlSignal_L, controlSignal_R);
}

void scenarioFour()
{
  double ang_speedL, ang_speedR, lin_speedL, lin_speedR, speedL, speedR, setSpeed;
  setSpeed = wallFrontController.update(distFront, 300);
  unsigned long timee = millis();
  
  get_AngularVelocities(speedL, speedR);
  ang_speedL = speedL;
  ang_speedR = speedR;
  get_LinearVelocities(speedL, speedR);
  lin_speedL = speedL;
  lin_speedR = speedR;
  float controlSignal_L = speedControllerL.update(lin_speedL, setSpeed);
  float controlSignal_R = speedControllerR.update(lin_speedR, setSpeed);
  driveForward(controlSignal_L, controlSignal_R);

  //save in sd
  saveToSD(timee, ang_speedL, ang_speedR, lin_speedL, lin_speedR, controlSignal_L, controlSignal_R);
}

void saveToSD(unsigned long t, double ang_L, double ang_R, double lin_L, double lin_R, byte Sig_L, byte Sig_R)
{

}

void setColor(uint8_t r, uint8_t g, uint8_t b) 
{
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, r,g,b); // Set color for each LED
  }
  strip.show(); // Update the LEDs
}

void loop() {
// put your main code here, to run repeatedly:
uint32_t ticks = i_R;
//Serial.println("ticks: "+ ticks);
  if(blinker)
  {
    setColor(0,255,0);
    unsigned long current_Time = millis();
    if(current_Time >= last_Time + 5)
    {
      //myICM.getAGMT();
      last_Time = current_Time;
      readIR(distLeft, distFrontLeft, distFront, distFrontRight, distFront);
      // Serial.println("Distance Left: " + String(distLeft) + " Distance Front Left: " + String(distFrontLeft) + 
      //                " Distance Front: " + String(distFront) + " Distance Front Right: " + String(distFrontRight) + 
      //                " Distance Right: " + String(distRight));
    }
    //scenerioOne();
    scenerioTwo();
    //scenerioThree();
    //scenerioFour();
    //driveForward(40,40);
    if(current_Time >= start_Time + 5000)
    {
      setColor(0,0,255);
      blinker = false;
      driveForward(0, 0);
      delayMicroseconds(100);
      driveBackward(0, 0);
      delay(3000);
    }
  }
  else 
  {
    setColor(255,0,0);
  }
}