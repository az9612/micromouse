/** Resources
  * https://docs.arduino.cc/learn/programming/bit-math
  * https://marsuniversity.github.io/ece387/FloodFill.pdf
  * http://craga89.github.io/Micromouse/
  * https://www.geeksforgeeks.org/set-clear-and-toggle-a-given-bit-of-a-number-in-c/
*/

// Floodfill
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include "src/CircularBufferQueue/CircularBufferQueue.h"
#include <EEPROM.h>

#include <ICM_20948.h>
//#include <>
#include <Adafruit_NeoPixel.h>
#include <AutoPID.h>

#define cs 5 

#define I2C_ADDRESS 0x3C



#define rows 16
#define cols 16

// Matrix macros
#define lineariseIndex(row, col) row* cols + col
#define delineariseRow(location) location / cols
//#define lineariseIndex(row, col) (rows - 1 - row)*cols + col
//#define delineariseRow(location) (rows - 1 - (location/ cols))
#define delineariseCol(location) location % cols
#define delineariseRow2(location) rows - 1 - (location / cols)


// Wall macros
#define distance(loc1, loc2) abs(delineariseRow(loc1) - delineariseRow(loc2)) + abs(delineariseCol(loc1) - delineariseCol(loc2))
#define markWall(location, direction) floodArray[location].neighbours |= 1 << direction
#define wallExists(location, direction) bitRead(floodArray[location].neighbours, direction)

// Neighbour macros
#define getNeighbourLocation(location, direction) (byte)((short)location + cellDirectionAddition[direction])  // Calculates the location of neighbour
#define getNeighbourDistanceIfAccessible(location, direction) floodArray[getNeighbourLocation(location, direction)].flood
#define getNeighbourDistance(location, direction) wallExists(location, direction) ? 255 : getNeighbourDistanceIfAccessible(location, direction)
#define getNeighbourDistanceIfAccessible2(location, direction) floodArray2[getNeighbourLocation(location, direction)].flood
#define getNeighbourDistance2(location, direction) wallExists(location, direction) ? 255 : getNeighbourDistanceIfAccessible2(location, direction)
// Direction macros
#define updateDirection(currentDirection, turn) *currentDirection = (*currentDirection + turn) % 4  // Updates the passed direction

#define north 0
#define east 1
#define south 2
#define west 3

#define rightTurn 1
#define uTurn 2
#define leftTurn 3

#define leftSensor 0
#define diagonalLeftSensor 1
#define centreSensor 2
#define diagonalRightSenson 3
#define rightSensor 6

#define Kp 250// 250
#define KI 10 //20
#define KD 15// 15 

// Define the LED strip configuration
#define LED_PIN    13  // Pin connected to the data input of the LED strip
#define NUM_LEDS   3 // Number of LEDs in the strip

double t;
String valueRow;
String valueCol;
String valueT;

int schedule = 0;
struct cell {
  byte flood;
  byte neighbours;
  byte visited;
  bool operator==(const cell& other) const {
    return flood == other.flood;
  }
  bool operator!=(const cell& other) const {
    return !(*this == other);  // Negation von ==
  }
};

cell floodArray[rows * cols];
cell floodArray2[rows * cols];  // This array stores the flood value and neighbour data for all the cells
cell floodArraySafe[rows * cols];
cell floodArraySafe2[rows * cols];
//byte targetCells[4];  // This array stores the target cells
byte targetCells[4] = { lineariseIndex(7, 7), lineariseIndex(7, 8), lineariseIndex(8, 7), lineariseIndex(8, 8) };
byte startCell = lineariseIndex(15, 0);  //
byte startDir = north;                   //
byte targetCells2 = { startCell };
CircularBufferQueue floodQueue(500);  // This queue stores the cells that need to be flooded
CircularBufferQueue floodQueue2(500);
byte currentCell, targetCell;

byte leftDir, currentDir, rightDir, nextLeftDir, nextDir, nextRightDir;


short cellDirectionAddition[4] = { -rows, 1, rows, -1 };  // The location of a neighbouring cell can be obtained using the values in this dictionary
byte updateDirectionTurnAmount[4] = { 0, rightTurn, uTurn, leftTurn };
byte targetScoreFromDirection[4] = { 0, 1, 2, 1 };

byte readingCellLoc, readingCellDistance, readingCellScore, minNeighbourDistance, targetRelativeDirection, targetScore;
byte distanceFromTarget = 1;
byte resetMaze = 0;

// Encoder and Oled Stuff
// long newPosition1 = 0, newPosition2 = 0;
// long oldPosition1 = 0, oldPosition2 = 0;
byte menu = 0;
short change;

byte* values[7] = { &startCell, &(targetCells[0]), &(targetCells[1]), &(targetCells[2]), &(targetCells[3]), &startDir, &resetMaze };

// Motor Control

ICM_20948_I2C myICM;

// #define I2C_ADDRESS 0x3C

// Create a NeoPixel object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);


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
double wallError;

byte buttonPin = 12;

// Vehicle control
byte motorL_in1 = 4;
byte motorL_in2 = 0;
byte encPinL = 26;

byte motorR_in1 = 17;
byte motorR_in2 = 33;
byte encPinR = 35;


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
unsigned long blinker_Time;

// Variables for encoder counts and speed
volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;
double currentSpeedLeft = 0;
double currentSpeedRight = 0;
const unsigned long speed_sampleTime = 20; // PID loop interval in milliseconds
const unsigned long wall_sampleTime = 20;
const unsigned long kinematics_sampleTime = 10;
double max_targetSpeed = 1.6;
double targetSpeed = 0.6;
double motorOutputL, motorOutputR;
double setSpeedLeft, setSpeedRight;
double setSpeedLeft0 = 0.17;
double setSpeedRight0 = 0.17;
double newPosition;

double setSpeed;
unsigned long lastTime = 0;
bool loopActive = false;

// Kinematics
double theta, posx, posy;
double orientation, orientation00, coordinateX, coordinateY;


// Initialize the PID controller


double prevErrorL = 0;
double prevErrorR = 0;
double prevErrorLL = 0;
double prevErrorRL = 0;
double prevErrorLR = 0;
double prevErrorRR = 0;
double integralL = 0, integralR= 0;
double setSpeedLeft1, setSpeedRight1;
double errorL = 0, errorR = 0, derivativeL = 0, derivativeR = 0;

byte nextCellx, nextCelly;

void setup() {
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
  /*
  if (!SD.begin(cs)) {
    log("Card Mount Failed");
    return;
  }
  log("SD Card Initialized Successfully!");
  writeFile(SD, "/test.txt", "Start \n");
  */
  initializeFloodArray();
  resetMazeValuesInEEPROM();
  initializeFloodArray2();
  resetMazeValuesInEEPROM2();
}
  

void loop() {
testIR();
setColor(0,100,0); // 
  if(blinker)
  {
    unsigned long current_Time = millis();
    if (current_Time >= blinker_Time + 5000)
    {
      bool areEqual = true;
      for (int i = 0; i < 256; i++) {
        if (floodArray[i] != floodArraySafe[i]) {
          areEqual = false;
          break;
        }
      }
      if (areEqual && schedule == 0) { //
        //log("Shortest Path");
        printFloodArray();
        initialiseDirections();
        while (currentCell != targetCells[0] && currentCell != targetCells[1] && currentCell != targetCells[2] && currentCell != targetCells[3]) {
        updateTargetCell();
        goToTargetCell();
        //currentCell = targetCell;
        }
        //log("Done!");

        schedule = 1;
      }
      if (schedule == 0) {
        //log("Started");
      
        char floodrow[80] = "\0";

        initialiseDirections();
        printFloodArray2();

        while (currentCell != targetCells[0] && currentCell != targetCells[1] && currentCell != targetCells[2] && currentCell != targetCells[3]) {

          //log("CurrentDir");
          //dlog(currentDir);
          byte row = delineariseRow(currentCell);
          byte col = delineariseCol(currentCell);
          t = millis();
          valueT = String(t);
          valueRow = String(row);
          valueCol = String(col);
          updateWalls();
          flood();
          flood2();
          updateTargetCell();
          goToTargetCell();
          floodArray[currentCell].visited = 1;
        }
        for (int i = 0; i < 256; i++) {
          floodArraySafe[i] = floodArray[i];
        }
        printFloodArray3();
        while (currentCell != targetCells2) {

          //log("CurrentDir");
          //dlog(currentDir);
          updateWalls();
          flood2();
          flood();
          //printFloodArray3();
          updateTargetCell2();
          floodArray[currentCell].visited = 1;
          goToTargetCell();
          //printFloodArray3();
          //logCurrentCell(currentCell);
          //log("TargetAbsoluteDir");
          //dlog(getTargetAbsoluteDirection(targetCell));
          //currentCell = targetCell;
          floodArray2[currentCell].visited = 1;
        }
        printFloodArray();
        updateMazeValuesInEEPROM();
        //log("Reached.");
        //log("-----");
        //log("Starting next Run");
        delay(1000);
      }
    } else {
    }
  }
}


void flood() {
  if (currentCell == targetCells[0] || currentCell == targetCells[1] || currentCell == targetCells[2] || currentCell == targetCells[3]) return;
  floodQueue.enqueue(currentCell);
  while (!floodQueue.isEmpty()) {
    readingCellLoc = *floodQueue.dequeue();
    if (isEnclosed(readingCellLoc)) continue;
    readingCellDistance = floodArray[readingCellLoc].flood;
    minNeighbourDistance = 255;
    for (byte i = 0; i < 4; i++) {
      minNeighbourDistance = min((int)minNeighbourDistance, (int)getNeighbourDistance(readingCellLoc, i));
    }
    if (minNeighbourDistance != readingCellDistance - 1) {
      floodArray[readingCellLoc].flood = minNeighbourDistance + 1;
      for (byte i = 0; i < 4; i++) {
        if (isNeighbourValid(readingCellLoc, i)) {
          if (!isDestination(getNeighbourLocation(readingCellLoc, i))) {
            floodQueue.enqueue(getNeighbourLocation(readingCellLoc, i));
          }
        }
      }
    }
  }
}

void flood2() {
  if (currentCell == targetCells2) return;
  floodQueue2.enqueue(currentCell);
  while (!floodQueue2.isEmpty()) {
    readingCellLoc = *floodQueue2.dequeue();
    if (isEnclosed2(readingCellLoc)) continue;
    readingCellDistance = floodArray2[readingCellLoc].flood;
    minNeighbourDistance = 255;
    for (byte i = 0; i < 4; i++) {
      minNeighbourDistance = min((int)minNeighbourDistance, (int)getNeighbourDistance2(readingCellLoc, i));
    }
    if (minNeighbourDistance != readingCellDistance - 1) {
      floodArray2[readingCellLoc].flood = minNeighbourDistance + 1;
      for (byte i = 0; i < 4; i++) {
        if (isNeighbourValid(readingCellLoc, i)) {
          if (!isDestination2(getNeighbourLocation(readingCellLoc, i))) {
            floodQueue2.enqueue(getNeighbourLocation(readingCellLoc, i));
          }
        }
      }
    }
  }
}

void setColor(uint8_t r, uint8_t g, uint8_t b) 
{
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, r,g,b); // Set color for each LED
  }
  strip.show(); // Update the LEDs
}

void driveForward(int velL, int velR)
{
    analogWrite(motorL_in2, 0);
    delayMicroseconds(100);
    analogWrite(motorL_in1, velL);
    analogWrite(motorR_in2, 0);
    delayMicroseconds(100);
    analogWrite(motorR_in1, velR);
}

void updateTargetCell() {
  minNeighbourDistance = getNeighbourDistance(currentCell, 0);
  targetScore = 3;
  double m = (double)minNeighbourDistance;
  for (byte i = 0; i < 4; i++) {
    if (!wallExists(currentCell, i)) {
      readingCellLoc = getNeighbourLocation(currentCell, i);
      readingCellDistance = getNeighbourDistance(currentCell, i);
      readingCellScore = targetScoreFromDirection[getTargetRelativeDirection(readingCellLoc)];
      if ((readingCellDistance < minNeighbourDistance) || ((readingCellDistance == minNeighbourDistance) && (readingCellScore < targetScore))) {
        minNeighbourDistance = readingCellDistance;
        targetScore = readingCellScore;
        targetCell = readingCellLoc;
      }
    }
  }
  targetRelativeDirection = getTargetRelativeDirection(targetCell);

  updateDirection(&nextLeftDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&nextDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&nextRightDir, updateDirectionTurnAmount[targetRelativeDirection]);

  distanceFromTarget = 1;
  
  // while (isNeighbourValid(targetCell, nextDir)) {
  //   readingCellLoc = getNeighbourLocation(targetCell, nextDir);
  //   if (isTunnel(readingCellLoc) && floodArray[readingCellLoc].flood == floodArray[targetCell].flood - 1) {
  //     targetCell = readingCellLoc;
  //     distanceFromTarget++;
  //   } else break;
  // }
  
}

void updateTargetCell2() {
  minNeighbourDistance = getNeighbourDistance2(currentCell, 0);
  targetScore = 3;
  double m = (double)minNeighbourDistance;
  for (byte i = 0; i < 4; i++) {
    if (!wallExists(currentCell, i)) {
      readingCellLoc = getNeighbourLocation(currentCell, i);
      readingCellDistance = getNeighbourDistance2(currentCell, i);
      readingCellScore = targetScoreFromDirection[getTargetRelativeDirection(readingCellLoc)];
      if ((readingCellDistance < minNeighbourDistance) || ((readingCellDistance == minNeighbourDistance) && (readingCellScore < targetScore))) {
        minNeighbourDistance = readingCellDistance;
        targetScore = readingCellScore;
        targetCell = readingCellLoc;
      }
    }
  }


  targetRelativeDirection = getTargetRelativeDirection(targetCell);

  updateDirection(&nextLeftDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&nextDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&nextRightDir, updateDirectionTurnAmount[targetRelativeDirection]);

  distanceFromTarget = 1;
  /*
  while (isNeighbourValid(targetCell, nextDir)) {
    readingCellLoc = getNeighbourLocation(targetCell, nextDir);
    if (isTunnel(readingCellLoc) && floodArray[readingCellLoc].flood == floodArray[targetCell].flood - 1) {
      targetCell = readingCellLoc;
      distanceFromTarget++;
    } else break;
  }
  */
}



void runPID2(){
  int64_t newPositionL = i_L;
  int64_t newPositionR = i_R;
  newPosition = ((double) newPositionL+(double)newPositionR)/2;
  double stepsPID = 225;
  double targetPosPID = newPosition + stepsPID;
  //double targetPos = oldPosition + steps;

  while ((newPosition < targetPosPID) && distFront > 27)
  {
    
      int64_t newPositionL = i_L;
      int64_t newPositionR = i_R;
      readIR(distLeft, distFrontLeft, distFront, distFrontRight, distRight);
      kinematics();
    
      newPosition = ((double) newPositionL+(double)newPositionR)/2;
    
      //targetPos = newPos steps;
      wallError = 0.0002 * (distLeft - distRight);
      
      if (distLeft > 60) {
        setColor(100,0,0);
        wallError = 0.0002 * ((25) - distRight);
      }
      if (distRight > 55) {
        wallError = 0.0002 * ((distLeft) - 25);
        setColor(100,0,0);
      }
      
      setSpeedLeft1 = setSpeedLeft0 - wallError;
      setSpeedRight1 = setSpeedRight0 + wallError;

        if (abs(targetPosPID-newPosition) < 150) {
          if (setSpeedLeft1 > 0.05) {
            setSpeedLeft1 = setSpeedLeft1 - 0.01;
            setSpeedRight1 = setSpeedRight1 -0.01;
        } 
        else if (setSpeedLeft1 < 0.2) {
            setSpeedLeft1 = setSpeedLeft1 + 0.0;
            setSpeedRight1 = setSpeedRight1 + 0.0;  
        }
      
      }
      errorL = setSpeedLeft1- currentSpeedLeft;
      errorR = setSpeedRight1- currentSpeedRight;

      integralL += errorL;
      integralR += errorR;
      
      derivativeL = errorL - prevErrorL;
      derivativeR = errorR - prevErrorR;

      prevErrorL = errorL;
      prevErrorR = errorR;
      
      motorOutputL = Kp * errorL  + KD * derivativeL + KI * integralL;
      motorOutputR = Kp * errorR + KD * derivativeR + KI * integralR;
      // Begrenzung der Ausgangswerte auf PWM-Bereich (0-255)
      motorOutputL = constrain(motorOutputL, 40, 150);
      motorOutputR = constrain(motorOutputR, 40, 150);
      driveForward(motorOutputL,motorOutputR);  
  }
  delay(20);
  driveStop();
  delay(1000);
}
 

 
void runPID3(){
  int64_t newPositionL = i_L;
  int64_t newPositionR = i_R;
  newPosition = ((double) newPositionL+(double)newPositionR)/2;
  double stepsPID = 225;
  double targetPosPID = newPosition + stepsPID;
  //double targetPos = oldPosition + steps;

  while ((newPosition < targetPosPID) && distFront > 27)
  {
    int64_t newPositionL = i_L;
    int64_t newPositionR = i_R;
    readIR(distLeft, distFrontLeft, distFront, distFrontRight, distRight);
    kinematics();
   
    newPosition = ((double) newPositionL+(double)newPositionR)/2;
  
    //targetPos = newPos steps;
    wallError = 0.0002 * (distLeft  - distRight);
    
    if (distLeft > 60) {
      wallError = 0.0002 * ((20) - distRight);
    }
     else if (distRight > 55) {
      wallError = 0.0002 * ((distLeft) - 20);
    }
    
    setSpeedLeft1 = setSpeedLeft0 *1.03 - wallError;
    setSpeedRight1 = setSpeedRight0 + wallError;

      if (abs(targetPosPID-newPosition) < 150) {
        if (setSpeedLeft1 > 0.05) {
          setSpeedLeft1 = setSpeedLeft1 - 0.0;
          setSpeedRight1 = setSpeedRight1 -0.0;
      } 
      else if (setSpeedLeft1 < 0.2) {
          setSpeedLeft1 = setSpeedLeft1 + 0.0;
          setSpeedRight1 = setSpeedRight1 + 0.0;  
      }
    }
    errorL = (targetPosPID- (double) newPositionL) - (targetPosPID - (double) newPositionR);
 
    integralL += errorL;
    
    derivativeL = errorL - prevErrorL;

    prevErrorL = errorL;
  
    errorR = 15 * errorL  + 0 * derivativeL + 0 * integralL;

    motorOutputL = setSpeedLeft0 - errorR;
    motorOutputR = setSpeedRight0 + errorR;

    // Begrenzung der Ausgangswerte auf PWM-Bereich (0-255)
    motorOutputL = constrain(motorOutputL, 38, 150);
    motorOutputR = constrain(motorOutputR, 38, 150);
    driveForward(motorOutputL,motorOutputR);
  
  }
  delay(20);
  driveStop();
}

void runPID1() {
   int64_t newPositionL = i_L;
  int64_t newPositionR = i_R;
  newPosition = ((double) newPositionL+(double)newPositionR)/2;
  double stepsPID = 225;
  double targetPosPID = newPosition + stepsPID;
  //double targetPos = oldPosition + steps;

  while (distFront > 30)
  {
    int64_t newPositionL = i_L;
    int64_t newPositionR = i_R;
    readIR(distLeft, distFrontLeft, distFront, distFrontRight, distRight);
    kinematics();
   
    newPosition = ((double) newPositionL+(double)newPositionR)/2;
  
    //targetPos = newPos steps;
    wallError = 0.0004 * (distLeft  - distRight);
    
    if (distLeft > 60) {
      wallError = 0.0004 * ((25) - distRight);
    }
     else if (distRight > 55) {
      wallError = 0.0004 * ((distLeft) - 25);
    }
    
    setSpeedLeft1 = setSpeedLeft0 - wallError;
    setSpeedRight1 = setSpeedRight0 + wallError;

      if (abs(targetPosPID-newPosition) < 150) {
        if (setSpeedLeft1 > 0.2) {
          setSpeedLeft1 = setSpeedLeft1 - 0.4;
          setSpeedRight1 = setSpeedRight1 -0.4;
      } 
      else if (setSpeedLeft1 < 0.2) {
          setSpeedLeft1 = setSpeedLeft1 + 0.0;
          setSpeedRight1 = setSpeedRight1 + 0.0;  
      }
    }
    errorL = setSpeedLeft1- currentSpeedLeft;
    errorR = setSpeedRight1- currentSpeedRight;

    integralL += errorL;
    integralR += errorR;
    
    derivativeL = errorL - prevErrorL;
    derivativeR = errorR - prevErrorR;

    prevErrorL = errorL;
    prevErrorR = errorR;
    
    motorOutputL = Kp * errorL  + KD * derivativeL + KI * integralL;
    motorOutputR = Kp * errorR + KD * derivativeR + KI * integralR;

    // Begrenzung der Ausgangswerte auf PWM-Bereich (0-255)
    motorOutputL = constrain(motorOutputL, 20, 150);
    motorOutputR = constrain(motorOutputR, 20, 150);
    driveForward(motorOutputL,motorOutputR);
  
  }
  delayMicroseconds(100);
  driveStop();
}


void goToTargetCell() {
  if (targetRelativeDirection == north) {
  } else if (targetRelativeDirection == west) {
    runPID1();
    setColor(0,100,100); // 
    turnLeft();
  } else if (targetRelativeDirection == south) {
    runPID1();
    setColor(100,100,0);
    turnRight();
    delay(1000);
    turnRight();
  } else if (targetRelativeDirection == east) {
    runPID1();
    setColor(100,0,100);
    turnRight();
  }
  runPID2();
  updateDirection(&leftDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&currentDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&rightDir, updateDirectionTurnAmount[targetRelativeDirection]);

  currentCell = targetCell;
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
  distFront = pow(distF,3)*5.9835e-09+pow(distF,2)*-2.0427e-05+distF*0.0306+13.6494;
  distRight = pow(distR,4)*2.1381e-12+pow(distR,3)*-1.3567e-08+pow(distR,2)*2.9813e-05+distR*-0.0195+13.1184+6;

  if (distL <130) {
    distLeft = pow(distL,2)*-7.5504e-04+distL*0.2449-4.0505;
  }
  if (distR <230) {
     distRight = distR*0.0388+1.29891;
  }
  /*
  distLeft = distL;
  distRight = distR;
  distFront = distF;
  */
}

void cntRPM_L()
{
  i_L++;
}

void cntRPM_R()
{
  i_R++;
}

void testIR() {
  readIR(distLeft, distFrontLeft, distFront, distFrontRight, distRight);
  Serial.print("distLeft");
  Serial.print(distLeft);
  Serial.print(",");
  Serial.print("distFront");
  Serial.print(distFront);
  Serial.print(",");
  Serial.print("distRight");
  Serial.print(distRight);
  Serial.print(",");
  Serial.print("distFront");
  Serial.print(distFront);
  Serial.println(",");
  delay(1000);
}

void updateWalls() {
   readIR(distLeft, distFrontLeft, distFront, distFrontRight, distRight);
  Serial.print("distLeft");
  Serial.print(distLeft);
  Serial.println(",");
  Serial.print("distFront");
  Serial.print(distFront);
  Serial.println(",");
  Serial.print("distRight");
  Serial.print(distRight);
  Serial.println(",");
  Serial.print("distFront");
  Serial.print(distFront);
  Serial.println(",");
  if (distLeft < 60) {  //Schwellenwert 55
    markWall(currentCell, leftDir);
    Serial.println(distLeft);
    if (isNeighbourValid(currentCell, leftDir)) {
      markWall(getNeighbourLocation(currentCell, leftDir), (leftDir + 2) % 4);
    }
  }
  if (distFront < 100) { //Schwellwert 120
    markWall(currentCell, currentDir);
    Serial.println(distFront);
    if (isNeighbourValid(currentCell, currentDir)) {
      markWall(getNeighbourLocation(currentCell, currentDir), (currentDir + 2) % 4);
    }
  }
  if (distRight < 55) { //Schwellenwert 48
    markWall(currentCell, rightDir);
    Serial.println(distRight);
    if (isNeighbourValid(currentCell, rightDir)) {
      markWall(getNeighbourLocation(currentCell, rightDir), (rightDir + 2) % 4);
    }
  }
}

bool isNeighbourValid(byte location, byte direction) {
  if (direction == north) return delineariseRow(location) > 0;
  else if (direction == east) return delineariseCol(location) < (cols - 1);
  //else if (direction == east) return delineariseCol(location) <= (cols);
  else if (direction == south) return delineariseRow(location) < (rows - 1);
  //else if (direction == south) return delineariseRow(location) <= (rows);
  else if (direction == west) return delineariseCol(location) > 0;
  //else if (direction == west) return delineariseCol(location) >= 0;
  return false;
}

byte getTargetAbsoluteDirection(byte target) {
  short diff = (short)target - (short)currentCell;
  if (diff == -rows) return north;
  if (diff == 1) return east;
  if (diff == rows) return south;
  if (diff == -1) return west;
  return 255;
}

byte getTargetRelativeDirection(byte target) {
  return (getTargetAbsoluteDirection(target) + 4 - currentDir) % 4;
}

bool isDestination(byte location) {
  return floodArray[location].flood == 0;
}

bool isDestination2(byte location) {
  return floodArray2[location].flood == 0;
}

bool isEnclosed(byte location) {
  // 15 is 00001111 in binary, which means that there are walls in 4 all 4 directions of the cell
  return floodArray[location].neighbours == 15;
}
bool isEnclosed2(byte location) {
  // 15 is 00001111 in binary, which means that there are walls in 4 all 4 directions of the cell
  return floodArray2[location].neighbours == 15;
}
bool isTunnel(byte location) {
  return (!wallExists(location, nextDir)) && wallExists(location, nextLeftDir) && wallExists(location, nextRightDir) && floodArray[location].visited;
}

void initialiseDirections() {
  currentCell = startCell;
  currentDir = startDir;
  leftDir = (currentDir + 3) % 4;
  rightDir = (currentDir + 1) % 4;
  nextLeftDir = leftDir;
  nextDir = currentDir;
  nextRightDir = rightDir;
}

//////////////////////////////////
///////////OLED SETUP////////////
////////////////////////////////

void updateMazeValuesFromEEPROM() {
  for (uint8_t i = 0; i < (rows * cols); i++) {
    floodArray[i].flood = EEPROM.read(i);
    if (i == 255) break;
  }
  for (uint8_t i = 0; i < (rows * cols); i++) {
    floodArray[i].neighbours = EEPROM.read((rows * cols) + (short)i);
    if (i == 255) break;
  }
  for (uint8_t i = 0; i < (rows * cols); i++) {
    floodArray[i].visited = EEPROM.read((2 * rows * cols) + (short)i);
    if (i == 255) break;
  }
  for (uint8_t i = 0; i < 6; i++) {
    *(values[i]) = EEPROM.read((3 * rows * cols) + (short)i);
  }
}

void updateMazeValuesInEEPROM() {
  for (uint8_t i = 0; i < (rows * cols); i++) {
    EEPROM.write(i, floodArray[i].flood);
    if (i == 255) break;
  }
  for (uint8_t i = 0; i < (rows * cols); i++) {
    EEPROM.write((rows * cols) + (short)i, floodArray[i].neighbours);
    if (i == 255) break;
  }
  for (uint8_t i = 0; i < (rows * cols); i++) {
    EEPROM.write((2 * rows * cols) + (short)i, floodArray[i].visited);
    if (i == 255) break;
  }
  for (uint8_t i = 0; i < 6; i++) {
    EEPROM.write((3 * rows * cols) + (short)i, *(values[i]));
  }
}

void resetMazeValuesInEEPROM() {
  for (int i = 0; i < (rows * cols); i++) {
    floodArray[i].flood = 255;
    for (byte j = 0; j < 4; j++) floodArray[i].flood = min((int)floodArray[i].flood, (int)distance(i, targetCells[j]));
    floodArray[i].neighbours = 0;
    floodArray[i].visited = 0;
    if (delineariseRow(i) == 0) markWall(i, north);
    if (delineariseCol(i) == 0) markWall(i, west);
    if (delineariseRow(i) == (rows - 1)) markWall(i, south);
    if (delineariseCol(i) == (cols - 1)) markWall(i, east);
    if (i == 255) break;
  }
  updateMazeValuesInEEPROM();
}

//////////////////////////////////
///////////OLED SETUP////////////
////////////////////////////////

void oledSetup() {
}

void updateEncoder() {
}

void displayMenu() {
}

void printFloodArray() {
  String line = "";
  //log("Flood");
  for (byte i = 0; i < 16; i++) {
    for (byte j = 0; j < 16; j++) {
      byte flood = floodArray[lineariseIndex(i, j)].flood;
      line += (String)flood;
      if (flood < 10) line += " , ";
      else line += ", ";
    }
    //log(line);
    line = "";
  }
}

void printFloodArray2() {
  for (byte i = 0; i < 16; i++) {
    for (byte j = 0; j < 16; j++) {
    }
  }
}

void printFloodArray3() {
  for (byte i = 0; i < 16; i++) {
    for (byte j = 0; j < 16; j++) {
    }
  }
}

void initializeFloodArray2() {
  for (int i = 0; i < rows * cols; i++) {
    floodArray2[i].flood = 255;  // Setze alle Zellen auf einen hohen Wert, z.B. 255
    floodArray2[i].visited = 0;
    floodArray2[i].neighbours = 0;
  }
}
void initializeFloodArray() {
  for (int i = 0; i < rows * cols; i++) {
    floodArray[i].flood = 255;  // Setze alle Zellen auf einen hohen Wert, z.B. 255
    floodArray[i].visited = 0;
    floodArray[i].neighbours = 0;
  }
}
void resetMazeValuesInEEPROM2() {
  for (int i = 0; i < (rows * cols); i++) {
    floodArray2[i].flood = 255;
    floodArray2[i].flood = min((int)floodArray2[i].flood, (int)distance(i, targetCells2));
    floodArray2[i].neighbours = 0;
    floodArray2[i].visited = 0;
    if (delineariseRow(i) == 0) markWall(i, north);
    if (delineariseCol(i) == 0) markWall(i, west);
    if (delineariseRow(i) == (rows - 1)) markWall(i, south);
    if (delineariseCol(i) == (cols - 1)) markWall(i, east);
    if (i == 255) break;
  }
  updateMazeValuesInEEPROM();
}

void blink()
{
  blinker = true;
  start_Time = millis();
  blinker_Time = millis();
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
  double wheelDistance = 0.091;
  double timer = (double)kinematics_sampleTime/1000;
  
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


  theta = theta + omega * timer;   // orientation
  // theta = omega * timer;   // orientation
  speed = (currentSpeedLeft + currentSpeedRight) / 2;
  speedx = speed * cos(theta);
  speedy = speed * sin(theta);
  posx = posx + (speedx * timer);
  posy = posy + (speedy * timer);
  orientation = theta;

  // Serial.print("speed:");
  // Serial.print(speed);
  // Serial.print(",");
  // Serial.print("speedx:");
  // Serial.print(speedx);
  // Serial.print(",");
  // Serial.print("speedy:");
  // Serial.print(speedy);
  // Serial.print(",");
  // Serial.print("posx:");
  // Serial.print(posx);
  // Serial.print(",");
  // Serial.print("posy:");
  // Serial.print(posy);
  // Serial.print(",");
  // posx = speedx * timer;
  // posy = speedy * timer;

}

void turnRight() {

  int64_t oldPositionL = i_L;
  int64_t oldPositionR = i_R;
  int stepsR = 85;
  int16_t targetPosR = (int) oldPositionL + stepsR;
  double integralLR = 0, integralRR = 0;
  double newPositionLRight, newPositionRRight;

  while (newPositionLRight < targetPosR)
  {
   
    newPositionLRight = int(i_L);
    newPositionRRight = int(i_R);

    
    kinematics();
    /*
    Serial.print("newPositionL");
    Serial.print(newPositionLRight);
    Serial.println(",");
    Serial.print("targetPos");
    Serial.print(targetPosR);
    Serial.println(",");
    
    double setSpeedLeftTurn = setSpeedLeft0;
    //double setSpeedRightTurn = setSpeedRight0;

    double errorLR = setSpeedLeftTurn - (currentSpeedLeft);
    //double errorR = setSpeedRightTurn - abs(currentSpeedRight);
    
    Serial.print("currentSpeedLeft");
    Serial.print(currentSpeedLeft);
    Serial.println(",");
    

    integralLR += errorLR;
    //integralR += errorR;

    double derivativeLR = errorLR - prevErrorLR;
    //double derivativeR = errorR - prevErrorR;

    prevErrorLR = errorLR;
    //prevErrorR = errorR;
    
    motorOutputL = Kp * errorLR        + KD * derivativeLR + KI * integralLR;
    //motorOutputR = Kp * errorR * 1.13 + KD * derivativeR + KI * integralR;

    // Begrenzung der Ausgangswerte auf PWM-Bereich (0-255)
    motorOutputL = constrain(motorOutputL, 40, 100);
    //motorOutputR = constrain(motorOutputR, 40, 100);
    */
    rotateRight(37,37);
  
  }
  delay(100);
  driveStop();
}

void turnLeft() {

  Serial.println("---------------------------Left------------------------");
  
  int64_t oldPositionL2 = i_L;
  int64_t oldPositionR2 = i_R;
  
  int64_t stepsL = 85;
  int64_t targetPosL = oldPositionR2 + stepsL;
  double integralLL = 0, integralRL = 0;
  int64_t newPositionLLeft = (i_L);
  int64_t newPositionRLeft = (i_R);
  //double targetPos = oldPosition + steps;
  driveForward(0,0);

  while (newPositionRLeft < targetPosL)
  {
    
    newPositionLLeft = (i_L);
    newPositionRLeft = (i_R);
   

    kinematics();
    /*
    Serial.print("newPositionRLeft");
    Serial.print(newPositionRLeft);
    Serial.print(",");
    Serial.print("targetPosL");
    Serial.print(targetPosL);
    Serial.println(",");
    
    //double setSpeedLeft1L = setSpeedLeft0;
    double setSpeedRight1L = setSpeedRight0;

    //double errorLL = setSpeedLeft1 - abs(currentSpeedLeft);
    double errorRL = setSpeedRight1L - (currentSpeedRight);
    
    Serial.print("currentSpeedLeft:");
    Serial.print(currentSpeedLeft);
    Serial.print(",");
    Serial.print("currentSpeedRight:");
    Serial.print(currentSpeedRight);
    Serial.println(",");
    
    //integralLL += errorLL;
    integralRL += errorRL;

    //double derivativeLL = errorLL - prevErrorLL;
    double derivativeRL = errorRL - prevErrorRL;

    //prevErrorLL = errorLL;
    prevErrorRL = errorRL;
    
    //motorOutputL = Kp * errorL        + KD * derivativeL + KI * integralL;
    motorOutputR = Kp * errorRL * 1.13 + KD * derivativeRL + KI * integralRL;
    */
    // Begrenzung der Ausgangswerte auf PWM-Bereich (0-255)
    //motorOutputL = constrain(motorOutputL, 40, 100);
    motorOutputR = constrain(motorOutputR, 40, 100);
    rotateLeft(37,37);  
  }
  delay(100);
  driveStop();

}
 
void rotateLeft(double vel1, double vel2)
{
  Serial.print("---------------------------rotate Right------------------------");
  analogWrite(motorL_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in2, vel1);
  analogWrite(motorR_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in1, vel2);
}

void rotateRight(double vel1, double vel2)
{
  analogWrite(motorL_in2, 0);
  delayMicroseconds(200);
  analogWrite(motorL_in1, vel1);
  analogWrite(motorR_in1, 0);
  delayMicroseconds(200);
  analogWrite(motorR_in2, vel2);
}

void driveStop()
{
  analogWrite(motorL_in1, 255);
  analogWrite(motorL_in2, 255);
  analogWrite(motorR_in1, 255);
  analogWrite(motorR_in2, 255);
}