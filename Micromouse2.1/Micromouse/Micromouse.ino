#include <ICM_20948.h>
//#include <>
#include <Adafruit_NeoPixel.h>
#include <AutoPID.h>

// Define the LED strip configuration
#define LED_PIN    13  // Pin connected to the data input of the LED strip
#define NUM_LEDS   3 // Number of LEDs in the strip

ICM_20948_I2C myICM;

// #define I2C_ADDRESS 0x3C

// Create a NeoPixel object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

int8_t state = 0; 

// Instantiate Kalman Filters for pitch, roll, and yaw
//KalmanFilter kalmanYaw;

// IMU data
double accX, accY, accZ;
double gyrX, gyrY, gyrZ;
double magX, magY, magZ;
double theta_gyr;
double sx, sy, velX, velY;
double yawMagnetometer;


// Infrared sensors
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

uint16_t  distFrontLeft, distFrontRight, distL, distR, distF;
double distRight, distLeft, distFront;
double target_distLeft = 28;
double target_distRight = 28; 
double target_distFront;

byte buttonPin = 12;

// Vehicle control
byte motorL_in1 = 4;
byte motorL_in2 = 0;
byte encPinL = 26;

byte motorR_in1 = 17;
byte motorR_in2 = 33;
byte encPinR = 35;

bool turnright, turnleft = true;

unsigned long last_Time = 0;
unsigned long start_Time;
volatile uint32_t i_L;
volatile uint32_t i_R;
int64_t cnt00_L;
int64_t cnt00_R;
int64_t cnt01_L;
int64_t cnt01_R;
int32_t n = pow(2,32)-1;

bool blinker = false;

// Time variables
unsigned long prevTime;
double dt;
unsigned long revTime;
unsigned long wall_pidTime;
unsigned long speed_pidTime;
unsigned long kinematicsTime;

// Variables for encoder counts and speed
volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;
double currentSpeedLeft = 0;
double currentSpeedRight = 0;
const unsigned long speed_sampleTime = 50; // PID loop interval in milliseconds
const unsigned long wall_sampleTime = 25;
const unsigned long kinematics_sampleTime = 10;
double max_targetSpeed = 1.6;
double targetSpeed = 0.6;
double outputValL, outputValR;
double setSpeedLeft, setSpeedRight;
double setSpeedLeft0 = 0.8;
double setSpeedRight0 = 0.8;

double setSpeed;
unsigned long lastTime = 0;
bool loopActive = false;

// Kinematics
double theta, posx, posy;
double orientation, orientation00, coordinateX, coordinateY;


// Initialize the PID controller
// Speed control
#define KP 1
#define KI 0
#define KD 0
#define KPr 1
#define KIr 0
#define KDr 0

AutoPID speedControllerL(&currentSpeedLeft, &setSpeedLeft0, &outputValL, 0.4, 1.6, KP, KI, KD);
AutoPID speedControllerR(&currentSpeedRight, &setSpeedRight0, &outputValR, 0.4, 1.6, KPr, KIr, KDr);

// Wall control
#define KPwl 0.01
#define KIwl 0
#define KDwl 0
#define KPwr 0.01
#define KIwr 0
#define KDwr 0

AutoPID wallLeftController(&distLeft, &target_distLeft, &setSpeedLeft, -0.1, 0.1, KPwl, KIwl, KDwl);
AutoPID wallRightController(&distRight, &target_distRight, &setSpeedRight, -0.1, 0.1, KPwr, KIwr, KDwr);

// Front control
#define KPf 1.688
#define KIf 0.6222
#define KDf 0.2539

AutoPID wallFrontController(&distFront, &target_distFront, &setSpeed, max_targetSpeed, 0.0, KPf, KIf, KDf);

struct grid
{
    double x;
    double y;
};
const int rows = 12;          // Number of rows (y-axis)
const int cols = 12;          // Number of columns (x-axis)
const double delta = 0.148;     // Distance between points
grid position[rows][cols];

byte nextCellx, nextCelly;

void setup() 
{
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


  // Fill the grid with (x, y) coordinates
  for (int i = 0; i < rows; i++) {       // Loop over rows (y-coordinates)
      for (int j = 0; j < cols; j++) {   // Loop over columns (x-coordinates)
          double x = j * delta;          // Calculate x-coordinate
          double y = i * delta;          // Calculate y-coordinate
          position[i][j].x = x;           // Store the pair in the grid
          position[i][j].y = y;
      }
  }

  strip.begin(); // Initialize the LED strip
  strip.show();  // Turn off all LEDs

  prevTime = millis();
  revTime = prevTime;
  start_Time = millis();
  Serial.begin(115200);

  speedControllerL.setTimeStep(speed_sampleTime);
  speedControllerR.setTimeStep(speed_sampleTime);
  wallLeftController.setTimeStep(wall_sampleTime);
  wallRightController.setTimeStep(wall_sampleTime);
  wallFrontController.setTimeStep(wall_sampleTime);
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

void loop() 
{
  // put your main code here, to run repeatedly:
  
  if(blinker)
  {
    
    unsigned long current_Time = millis();
    if(current_Time >= kinematicsTime + kinematics_sampleTime)
    {
      kinematicsTime = current_Time;
      kinematics();                   // ideally kalman fusion with gyro+theta and accel+posx/posy
      //readIMU(accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ); 
      coordinateX = posx;
      coordinateY = posy;
      //coordinateX = complemantaryFilter(0.05, coordinateX, posx, sx);
      //coordinateY = complemantaryFilter(0.05, coordinateY, posy, sy);
      // orientation += complemantaryFilter(0.05, theta, theta_gyr);
      // coordinateX += complemantaryFilter(0.05, posx, sx);
      // coordinateY += complemantaryFilter(0.05, posy, sy);
    }
    if(current_Time >= wall_pidTime + wall_sampleTime)
    {
      wall_pidTime = current_Time;
      readIR(distLeft, distFrontLeft, distFront, distFrontRight, distRight);
      wallLeftController.run();
      wallRightController.run();
      //setSpeedLeft0 = 0.8;
      //setSpeedRight0 = 0.8;
      Serial.print("setSpeedLeft:");
      Serial.print(setSpeedLeft0);
      //Serial.print(",");  // Tab als 
      //Serial.print("setSpeedRight:");
      //Serial.print(setSpeedRight0);
      Serial.print(","); 
      Serial.print("currentSpeedLeft:");
      Serial.print(currentSpeedLeft);
      Serial.print(","); 
      Serial.print("currentSpeedRight:");
      Serial.print(currentSpeedRight);
      Serial.print(","); 
      // Serial.print("distLeft:");
      // Serial.print(distLeft);
      // Serial.print(","); 
      // Serial.print("distRight:");
      // Serial.println(distRight);  // Letzte Zahl mit println, um die Zeile abzuschließen
      
      // Serial.print("Left: " + String(setSpeedLeft));
      // Serial.printlln("");
      // Serial.print(" Right: " + String(setSpeedRight));
      // Serial.println("");
      // Serial.print("Left Speed: " + String(currentSpeedLeft));
      // Serial.println("");
      // Serial.print(" Right Speed: " + String(currentSpeedRight));
      // Serial.println("");
      // Serial.print(" IR Left:" + String(distLeft));
      // Serial.println("");
      // Serial.print("IR Right: " + String(distRight));
     

      // Serial.println("Distance Left: " + String(distLeft) + " Distance Front Left: " + String(distFrontLeft) + 
      //                " Distance Front: " + String(distFront) + " Distance Front Right: " + String(distFrontRight) + 
      //                " Distance Right: " + String(distRight));
    }
    if(coordinateX >= position[nextCellx][nextCelly].x && coordinateY >= position[nextCellx][nextCelly].y)
    {
      bool wallRight, wallFront, wallLeft;
      if(distRight > 4) 
      {
        wallRight = false;
        state = 2;
        
      }
      else              
      {
        wallRight = true;
      }

      if(distFront > 4) 
      {
        wallFront = false;
        //state = 1;
      }
      else              
      {
        wallFront = true;
      }
      if(distLeft > 4)  
      {
        wallLeft = false;
        state = 3;
      }
      else              
      {
        wallLeft = true;
      }
      //nextCell/direction = flooFill(wallRight, wallFront, wallLeft);
      updatePosition(wallRight, wallFront, wallLeft);

      // what condition to turn?
      
    }

    int state = 1;
    state = stateMachine(state, current_Time);

    if(state == 0)
    {
      setColor(0, 0, 255);
    }
    else if(state == 1)
    {
      setColor(0, 255, 255);
    }
    else if(state == 2)
    {
      setColor(255, 0, 255);
    }

    // if(current_Time >= start_Time + 5000)
    // {
    //   setColor(0, 0, 255);
    //   blinker = false;
    //   delay(100);
    //   driveForward(0, 0);
    // }
  }
  else 
  {
    setColor(255,0,0);
    stateMachine(0, 0);
  }
}

int stateMachine(int state, unsigned long current_Time)
{
  switch(state)
  {
    case 0:     //Calibrate IR and Position
    sx = 0.0;
    sy = 0.0;
    theta_gyr = 0.0;
    posx = 0.0;
    posy = 0.0;
    theta = 0.0;

    setColor(0,255,0);
    return 1;

    case 1:     //Vehicle Speed
    {
      if(current_Time >= speed_pidTime + speed_sampleTime)
      {
      speed_pidTime = current_Time;

      speedControllerL.run();
      speedControllerR.run();
      Serial.print("OutputLpre:" + String(outputValL));
      Serial.print("OutputRpre:" + String(outputValR));
      outputValL = map(outputValL, 0.4, 1.6, 35, 255);
      outputValR = map(outputValR, 0.4, 1.6, 35, 255);
      outputValL = constrain(outputValL,35,255);
      outputValR = constrain(outputValR,35,255);

      Serial.print("OutputL:" + String(outputValL));
      Serial.println("OutputR:" + String(outputValR));
      }
      if (outputValL >= 0 && outputValR >= 0)
      {
        driveForward(outputValL, outputValR);
      }
      else if (outputValL < 0 && outputValR < 0)
      {
        driveBackward(abs(outputValL), abs(outputValR));
      }
      return 1;
    }
    case 2:    // Rotate vehicle right
    {
      if(turnright == true)
      {
        orientation00 = orientation;
        setSpeedLeft, setSpeedRight = targetSpeed;
        return 0;
      }
      else if (turnright == false)
      {
        speedControllerL.run();
        speedControllerR.run();
        rotateRight(outputValL, outputValR);
        return 3;
      }
    }
    case 3:     // Rotate vehicle left
    {
      if(turnleft == true)
      {
        orientation00 = orientation;
        setSpeedLeft, setSpeedRight = targetSpeed;
        return 0;
      }
      else if (turnleft == false)
      {
        speedControllerL.run();
        speedControllerR.run();
        rotateLeft(outputValL, outputValR);
        return 3;
      }
    }
    case 4:       
    {
      driveStop();
      return 1;
    }
    default:
    return 0;
  }
}

void updatePosition(bool wallRight, bool wallFront, bool wallLeft)
{
  if(orientation > -10 && orientation < 10)
  {
    if(wallRight) nextCellx++;
    else if(wallFront) nextCelly++;
    else if(wallLeft) nextCellx--;
  }
  else if(orientation > 80 && orientation < 100)
  {
    if(wallRight) nextCelly--;
    else if(wallFront) nextCellx++;
    else if(wallLeft) nextCelly++;
  }
  else if(orientation > 170 && orientation < 190 || orientation > -190 && orientation < -170)
  {
    if(wallRight) nextCellx--;
    else if(wallFront) nextCelly--;
    else if(wallLeft) nextCellx++;
  }
  else if(orientation > -90 && orientation < -70)
  {
    if(wallRight) nextCelly++;
    else if(wallFront) nextCellx--;
    else if(wallLeft) nextCelly--;
  }
}
/*
void controlOrientation() //future shit
{
  readIMU(accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ);
  yawMagnetometer = atan2(magY, magX) * 180 / PI;
  if (yawMagnetometer < 0) yawMagnetometer += 360; // Ensure 0-360 range
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  double filteredYaw = kalmanYaw.update(yawMagnetometer, gyrZ, dt);
}*/

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
  dtime = dtime/1000.0; 
  double speed_R, speed_L;
  uint8_t ticks_per_rot = 140;
  double radiansPerCount = 2 * PI / ticks_per_rot; // Winkel pro Count
  double zwischenrechnerL = (double)cnt01_L*(double)radiansPerCount;
  double zwischenrechnerR = (double)cnt01_R*(double)radiansPerCount;
  speed_L = zwischenrechnerL / dtime;
  speed_R = zwischenrechnerR / dtime;
  cnt01_L = cnt00_L;
  cnt01_R = cnt00_R;
  in1 = speed_L;
  in2 = speed_R;
}

void get_LinearVelocity(double& speed_L, double& speed_R)
{
  double omega_L, omega_R ;
  get_AngularVelocities(omega_L, omega_R); 
  double r = 0.019; 
  speed_L = r * omega_L; // m/s linear speed
  speed_R = r * omega_R;
  //Serial.println("SpeedL: " + String(speed_L) + " SpeedR: " + String(speed_R));
}

void get_LinearVelocities(double& speed_L, double& speed_R)
{
  double omega_L = speed_L;
  double omega_R = speed_R; 
  double r = 0.019; 
  speed_L = r * omega_L; // m/s linear speed
  speed_R = r * omega_R;
  
  //Serial.println("omegaL: " + String(omega_L) + " omegaR: " + String(omega_R));
  //Serial.println("LinSpeedL: " + String(speed_L) + " LinSpeedR: " + String(speed_R));
}


void kinematics()
{
  double omega, speed, speedx, speedy; 
  double wheelDistance = 0.06;
  double timer = kinematics_sampleTime/1000;
  
  get_AngularVelocities(currentSpeedLeft, currentSpeedRight);
  get_LinearVelocities(currentSpeedLeft, currentSpeedRight);
  
  if(currentSpeedLeft >= currentSpeedRight)
  {
    omega = (currentSpeedLeft - currentSpeedRight) / wheelDistance;
  }
  else
  {
    omega = (currentSpeedRight - currentSpeedLeft) / wheelDistance;
  }

 
  theta += omega * timer;   // orientation
  // theta = omega * timer;   // orientation
  speed = (currentSpeedLeft + currentSpeedRight) / 2;
  speedx = speed * cos(theta);
  speedy = speed * sin(theta);
  posx += speedx * timer;
  posy += speedy * timer;
  // posx = speedx * timer;
  // posy = speedy * timer;

}

double complemantaryFilter(double alpha, double value, double value1, double value2)
{
  value = alpha*(value + value1)+(1-alpha)*value2; // a(theta+theta_gyr) + (1-a)*theta_kine
  return value;
}

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

  distLeft = pow(distL,4)*-1.7647e-12+pow(distL,3)*1.4833e-08+pow(distL,2)*-3.3084e-05+distL*0.0320+9.4376;
  distFront = pow(distF, 5) * 8.9719e-15 + pow(distF, 4) * -5.7233e-11 + pow(distF, 3) * 1.3600e-07 
              + pow(distF, 2) * -1.4151e-04 + distF * 0.0707 + 13.8345;
  distRight = pow(distR,4)*2.1381e-12+pow(distR,3)*-1.3567e-08+pow(distR,2)*2.9813e-05+distR*-0.0195+13.1184;
}

void driveBackward(double velL, double velR)
{
  analogWrite(motorL_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in2, velL);
  analogWrite(motorR_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in2, velR);
}

void driveStop()
{
  digitalWrite(motorL_in1, 0);
  digitalWrite(motorL_in2, 0);
  digitalWrite(motorR_in1, 0);
  digitalWrite(motorR_in2, 0);
}

void driveForward(double velL, double velR)
{
  analogWrite(motorL_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in1, velL);
  analogWrite(motorR_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in1, velR);
}

void rotateRight(double vel1, double vel2)
{
  analogWrite(motorL_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in2, vel1);
  analogWrite(motorR_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in1, vel2);

  if(orientation >= orientation00 + 90)
  {
    turnright = true;
  }

//double gyro = myICM.gyrZ();
// double gyro = 0;

// angle += gyro*0.005;
// if(angle >= 90)
// {
//   turnright = true;
//   angle = 0;
// }
}

void rotateLeft(double vel1, double vel2)
{
  analogWrite(motorL_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in1, vel1);
  analogWrite(motorR_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in2, vel2);
  
  if(orientation <= orientation00 - 90)
  {
    turnleft = true;
  }

//double gyro = myICM.gyrZ();
// double gyro = 0;
// angle += gyro*0.005;
// if(angle >= -90)
// {
//   turnleft = true;
//   angle = 0;
// }
}

void cntRPM_L()
{
  i_L++;
}

void cntRPM_R()
{
  i_R++;
}


void setColor(uint8_t r, uint8_t g, uint8_t b) 
{
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, r,g,b); // Set color for each LED
  }
  strip.show(); // Update the LEDs
}


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

void readIMU(double& x_acc, double& y_acc, double& z_acc, double& x_gyr, double& y_gyr, double& z_gyr, double& x_mag, double& y_mag, double& z_mag)
{
  double timer = kinematics_sampleTime/1000;
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
    
    theta_gyr += timer*gyrZ;
    // theta_gyr = timer*gyrZ;
    orientation = complemantaryFilter(0.05, orientation, theta, theta_gyr);
    orientation += complemantaryFilter(0.05, orientation, theta, theta_gyr);
    
    sx += x_acc * cos(orientation) * pow(timer, 2) + velX * timer;
    sy += x_acc * sin(orientation) * pow(timer, 2) + velY * timer; 
    // sx = x_acc * cos(orientation) * pow(timer, 2) + velX * timer;
    // sy = x_acc * sin(orientation) * pow(timer, 2) + velY * timer; 
    velX = x_acc * cos(orientation) * timer; 
    velY = x_acc * sin(orientation) * timer;
  }
}

void blink()
{
  blinker = true;
  start_Time = millis();
}