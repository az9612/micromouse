/** Resources
  * https://docs.arduino.cc/learn/programming/bit-math
  * https://marsuniversity.github.io/ece387/FloodFill.pdf
  * http://craga89.github.io/Micromouse/
  * https://www.geeksforgeeks.org/set-clear-and-toggle-a-given-bit-of-a-number-in-c/
*/

#include "src/CircularBufferQueue/CircularBufferQueue.h"
#include <EEPROM.h>



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

#define threshold 200

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
long newPosition1 = 0, newPosition2 = 0;
long oldPosition1 = 0, oldPosition2 = 0;
byte menu = 0;
short change;

byte* values[7] = { &startCell, &(targetCells[0]), &(targetCells[1]), &(targetCells[2]), &(targetCells[3]), &startDir, &resetMaze };

void setup() {
  Serial.begin(19200);
  initializeFloodArray();
  resetMazeValuesInEEPROM();
  initializeFloodArray2();
  resetMazeValuesInEEPROM2();
}

void loop() {
  bool areEqual = true;
  for (int i = 0; i < 256; i++) {
    if (floodArray[i] != floodArraySafe[i]) {
      areEqual = false;
      break;
    }
  }
  if (areEqual && schedule == 0) { //
    log("Shortest Path");
    printFloodArray();
    initialiseDirections();
    while (currentCell != targetCells[0] && currentCell != targetCells[1] && currentCell != targetCells[2] && currentCell != targetCells[3]) {
    updateTargetCell();
    goToTargetCell();
    //currentCell = targetCell;
    setColor(delineariseCol(currentCell), delineariseRow2(currentCell), 'o');
    }
    setColor(delineariseCol(currentCell), delineariseRow2(currentCell), 'o');
    setText(delineariseCol(currentCell), delineariseRow2(currentCell), "Sol");
    log("Done!");

    schedule = 1;
  }
  if (schedule == 0) {
    log("Started");
    setColor(0, 0, 'R');
    setColor(8, 8, 'G');
    setColor(7, 8, 'G');
    setColor(8, 7, 'G');
    setColor(7, 7, 'G');
    char floodrow[80] = "\0";

    initialiseDirections();
    printFloodArray2();

    while (currentCell != targetCells[0] && currentCell != targetCells[1] && currentCell != targetCells[2] && currentCell != targetCells[3]) {

      //log("CurrentDir");
      //dlog(currentDir);

      updateWalls();
      flood();
      flood2();
      //printFloodArray3();
      updateTargetCell();
      log("updateTargetCell");
      setColor(delineariseCol(currentCell), delineariseRow2(currentCell), 'B');

      //setText(delineariseCol(currentCell),delineariseRow2(currentCell),String(floodArray[currentCell].flood));
      goToTargetCell();
      //logCurrentCell(currentCell);
      //log("TargetAbsoluteDir");
      //dlog(getTargetAbsoluteDirection(targetCell));
      //currentCell = targetCell;
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
      setColor(delineariseCol(currentCell), delineariseRow2(currentCell), 'B');
      floodArray[currentCell].visited = 1;
      //setText(delineariseCol(currentCell),delineariseRow2(currentCell),String(floodArray[currentCell].flood));
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
    log("Reached.");
    log("-----");
    ackReset();
    log("Starting next Run");
    delay(1000);
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

void updateTargetCell() {
  minNeighbourDistance = getNeighbourDistance(currentCell, 0);
  targetScore = 3;
  double m = (double)minNeighbourDistance;
  log("Target score");
  dlog(m);
  for (byte i = 0; i < 4; i++) {
    if (!wallExists(currentCell, i)) {
      readingCellLoc = getNeighbourLocation(currentCell, i);
      readingCellDistance = getNeighbourDistance(currentCell, i);
      readingCellScore = targetScoreFromDirection[getTargetRelativeDirection(readingCellLoc)];
      if ((readingCellDistance < minNeighbourDistance) || ((readingCellDistance == minNeighbourDistance) && (readingCellScore < targetScore))) {
        minNeighbourDistance = readingCellDistance;
        targetScore = readingCellScore;
        targetCell = readingCellLoc;
        log("TargetCell: ");
        logCurrentCell(targetCell);
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
  log("Target score");
  dlog(m);
  for (byte i = 0; i < 4; i++) {
    if (!wallExists(currentCell, i)) {
      readingCellLoc = getNeighbourLocation(currentCell, i);
      readingCellDistance = getNeighbourDistance2(currentCell, i);
      readingCellScore = targetScoreFromDirection[getTargetRelativeDirection(readingCellLoc)];
      if ((readingCellDistance < minNeighbourDistance) || ((readingCellDistance == minNeighbourDistance) && (readingCellScore < targetScore))) {
        minNeighbourDistance = readingCellDistance;
        targetScore = readingCellScore;
        targetCell = readingCellLoc;
        log("TargetCell: ");
        logCurrentCell(targetCell);
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


void goToTargetCell() {
  log("TargetRelativeDircetion:");
  dlog(targetRelativeDirection);
  if (targetRelativeDirection == north) {
  } else if (targetRelativeDirection == west) {
    turnLeft();
  } else if (targetRelativeDirection == south) {
    turnLeft();
    turnLeft();
  } else if (targetRelativeDirection == east) {
    turnRight();
  }
  moveForward();
  updateDirection(&leftDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&currentDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&rightDir, updateDirectionTurnAmount[targetRelativeDirection]);

  currentCell = targetCell;
}

void updateWalls() {
  if (wallLeft()) {
    markWall(currentCell, leftDir);
    log("Wall left");
    if (isNeighbourValid(currentCell, leftDir)) {
      markWall(getNeighbourLocation(currentCell, leftDir), (leftDir + 2) % 4);
    }
  }
  if (wallFront()) {
    markWall(currentCell, currentDir);
    log("Wall front");
    if (isNeighbourValid(currentCell, currentDir)) {
      markWall(getNeighbourLocation(currentCell, currentDir), (currentDir + 2) % 4);
    }
  }
  if (wallRight()) {
    markWall(currentCell, rightDir);
    log("Wall right");
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
  log("Flood");
  for (byte i = 0; i < 16; i++) {
    for (byte j = 0; j < 16; j++) {
      byte flood = floodArray[lineariseIndex(i, j)].flood;
      line += (String)flood;
      if (flood < 10) line += " , ";
      else line += ", ";
    }
    log(line);
    line = "";
  }
}

void printFloodArray2() {
  for (byte i = 0; i < 16; i++) {
    for (byte j = 0; j < 16; j++) {

      setText(j, 15 - i, String(floodArray[lineariseIndex(i, j)].flood));
    }
  }
}

void printFloodArray3() {
  for (byte i = 0; i < 16; i++) {
    for (byte j = 0; j < 16; j++) {

      setText(j, 15 - i, String(floodArray2[lineariseIndex(i, j)].flood));
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