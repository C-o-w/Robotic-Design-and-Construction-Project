// Main change to take note of, using PrintMessage() and PrintMessageLn() instead of print to allow it to switch between bluetooth and serial depending which is enabled with variables (line 332-407)
// Debug and DebugLn also included, 0 for regular debug, 1 for verbose debug


// USES ESP32 VERSION 2.0.17, WILL REQUIRE A ROLLBACK IN THE BOARDS MANAGER. THIS WAS DUE TO THE LIBRARY FOR THE MOTOR CONTROLLER NOT BEING UPDATED, AND NO SUTABLE REPLACEMENT BEING FOUND
// https://github.com/espressif/arduino-esp32

#include <Arduino.h>
#include <queue>
#include <vector>
#include <math.h>

#include <TB6612FNG.h>        //Motor controller library https://github.com/vincasmiliunas/ESP32-Arduino-TB6612FNG/blob/master/examples/TwoMotors/TwoMotors.ino
#include <BluetoothSerial.h>  //ESP32 bluetooth library https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial/examples

//Bluetooth Setup debug text from the library
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#include <Wire.h>

#define TCA9548A_ADDR 0x70  // Address of the multiplexer
#define SENSOR_ADDR 0x40    // Address of the Sharp GP2Y0E02B sensor

//Creation of the bluetooth object
BluetoothSerial SerialBT;

// Control Variables
int Start = 1;
int LoopDelay = 1;

// defines pins numbers
const int trigPin0 = 21;
const int echoPin0 = 22;
const int trigPin1 = 4;
const int echoPin1 = 5;
const int trigPin2 = 18;
const int echoPin2 = 19;

// defines variables
long duration;
int distance;

//Ints for storing wall detection
int WallFront;
int WallBack;
int WallLeft;
int WallRight;

//Distance Thresholds in mm
int UltrasoundFrontThreshold = 50;
int UltrasoundRightThreshold = 100;
int UltrasoundLeftThreshold = 100;
int DistanceErrorThreshold = 300;

//Array to store the IR values
int DistanceValues[4];

//Motor Setup
#define MotorAPWM 13  //Motor Pins
#define MotorA1 14
#define MotorA2 12
#define MotorBPWM 33
#define MotorB1 26
#define MotorB2 25
#define STBY 27

//Creation of motor controller objects for both motors

Tb6612fng motors(STBY, MotorA1, MotorA2, MotorAPWM, MotorB1, MotorB2, MotorBPWM);

//Variables for the rotary encoders
#define LeftOutputA 35  //Encoder Pins
#define LeftOutputB 34
#define RightOutputA 15
#define RightOutputB 2

int LeftCounter = 0;

int RightCounter = 0;

volatile int DistanceTravelled = 0;
int DistanceOffsetX = 0;
int DistanceOffsetY = 0;
String MoveAxis = "X";

int WheelDiameter = 47;        //In mm
float WheelCircumference = 0;  //Calculated in setup
int CountsPerRotation = 300;
int EncoderDifference;

int WheelGap = 75;  //In mm

bool Turning = false;

//Command Setup
String InputCommand;                //Full command inputted
int CommandIndexA;                  //Position of first colon
int CommandIndexB;                  //Position of second colon
String CurrentCommand;              //First section of the inputted string (before the first colon)
String CommandInputA;               //Second section of the inputted string (between the colons)
String CommandInputB;               //Third section of the inputted strong (after the last colon)
float CommandIntA;                  //Used if the first input is an int
float CommandIntB;                  //Used if the second input is an int
String OutputString = ("INITIAL");  //Used for data output

//Variables to declare which outputs to use. Bluetooth much cleaner than serial due to problems with ledc
int BluetoothEnabled = 1;
int SerialEnabled = 1;
int DebugEnabled = 1;
int VerboseEnabled = 1;

int DebugInt;    //For passing ints to debug when needed
int DebugFloat;  //For passing floats to debug when needed

int MovementCounter = 0;
char MovementList[] = {};  //Forwards = A, TurnLeft = B, TurnRight = C, Back = D

#define N1 50       // Size of the maze (20x20) ROW
#define N2 37       //                          COL
#define INF 999999  // Infinity, representing unvisited cells
int CellSize = 40;  //Size of one cell in MM

// Directions for moving in the maze (up, down, left, right)
int dRow[] = { -1, 1, 0, 0 };  // Row direction for up and down
int dCol[] = { 0, 0, -1, 1 };  // Column direction for left and right

// Maze: 0 for unknown, 1 for free space, 2 for wall, temporoary simulated map, in future fill with 0s as base and fill information as found
int maze[N1][N2] = {};


int startRow = 2;
int startCol = 2;  // Starting point (top-left)
int endRow = 39;
int endCol = 39;  // Ending point (bottom-right)

std::vector<std::pair<int, int>> path;

// Structure to represent a cell in the maze
struct Cell {
  int row, col;
  int dist;  // Distance from the start cell
  bool operator>(const Cell& other) const {
    return dist > other.dist;  // Min-heap by distance
  }
};

struct encoder {  //define encoder construct
  volatile const uint8_t PIN;
  volatile uint32_t numberRotation;
  volatile bool triggered;
};

encoder EncoderX = { RightOutputB, 0, false };  //create encoder instance

void IRAM_ATTR EncoderXInterrupt() {
  EncoderX.numberRotation += 1;
  EncoderX.triggered = true;
}

encoder EncoderY = { LeftOutputA, 0, false };  //create encoder instance

void IRAM_ATTR EncoderYInterrupt() {
  EncoderY.numberRotation += 1;
  EncoderY.triggered = true;
}

enum Direction { NORTH,
                 EAST,
                 SOUTH,
                 WEST };
Direction currentDirection = NORTH;  // Initial orientation of the robot

int CurrentX = 0;
int CurrentY = 0;

void RotateTo(Direction targetDirection);                   // Running into compilation error, declaring a function prototype fixed them
void PlaceWall(Direction wallDirection, int wallDistance);  // Same method for placewall

void setup() {
  pinMode(trigPin1, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT);   // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT);   // Sets the echoPin as an Input

  pinMode(EncoderX.PIN, INPUT_PULLUP);
  attachInterrupt(EncoderX.PIN, EncoderXInterrupt, FALLING);

  pinMode(EncoderY.PIN, INPUT_PULLUP);
  attachInterrupt(EncoderY.PIN, EncoderYInterrupt, FALLING);

  WheelCircumference = WheelDiameter * M_PI;

  Wire.begin();
  motors.begin();                //Begins the motor object
  Serial.begin(9600);            //Serial began at a baud rate of 9600, baud rate doesnt matter particularly but must match on the terminal
  SerialBT.begin("MicroMouse");  //Bluetooth device name
  PrintMessageLn("Setup Complete");
  delay(100);
  InitializeMaze();
  PrintMessageLn("Maze Initialized");
}

void loop() {  //Just encoder counting and checking for commands, all maze solving is controlled by FollowLeft and SolveMaze, controlled by commands FollowLeft:: and SolveMaze::
  CheckCommand();
  PollSensor();
  //PrintSensor();

  PrintMessage(String(CurrentX));
  PrintMessage(" : ");
  PrintMessageLn(String(CurrentY));
  for (int i = 0; i < 3; i++) {
    Serial.print("Distance at channel ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(DistanceValues[i]);
    Serial.println(" mm");
  }
  /*
  FindWall();
  PrintMaze();
  MoveSpace();
*/

  if (EncoderX.triggered) {
    DebugLn("Encoder X has triggered: ", 1);
    DebugLn(String(EncoderX.numberRotation), 1);
    EncoderX.triggered = false;
  }

  if (EncoderY.triggered) {
    DebugLn("Encoder Y has triggered: ", 1);
    DebugLn(String(EncoderY.numberRotation), 1);
    EncoderY.triggered = false;
  }

  delay(100);
}

//CHECKCOMMAND AND READCOMMAND ARE COMPLETELY UNNECESSARY TO THE FUNCTIONALITY OF THE DEVICE, PURELY DEBUG
void CheckCommand() {  //Checks if a command has been sent to either the serial or bluetooth ports, not sure what happens if both are sent at once, please dont test it
  if (Serial.available() != 0) {
    InputCommand = Serial.readString();
    CommandIndexA = InputCommand.indexOf(":");
    if (CommandIndexA == -1) {
      InputCommand = InputCommand + ":000:";
    }
    ReadCommand();
  }
  if (SerialBT.available() != 0) {
    InputCommand = SerialBT.readString();
    CommandIndexA = InputCommand.indexOf(":");
    if (CommandIndexA == -1) {
      InputCommand = InputCommand + ":000:";
    }
    ReadCommand();
  }
}

void ReadCommand() {  //Detects the markers in the code and separates the sections, then decides which function to trigger.
  PrintMessageLn(InputCommand);
  CommandIndexA = InputCommand.indexOf(":");  //Error with detecting commands without any colons at the end
  if (CommandIndexA > -1) {
    CommandIndexB = InputCommand.indexOf(":", (CommandIndexA + 1));
    CurrentCommand = InputCommand.substring(0, CommandIndexA);
  } else {
    CurrentCommand = String(InputCommand);
    CommandIndexB = -1;
  }
  if (CommandIndexA > -1) {  //
    if (CommandIndexB > -1) {
      CommandInputA = InputCommand.substring((CommandIndexA + 1), (CommandIndexB));
      CommandInputB = InputCommand.substring((CommandIndexB + 1));
    } else {
      CommandInputA = InputCommand.substring((CommandIndexA + 1));
      CommandInputB = "0";
    }
  } else {
    CommandInputA = InputCommand.substring((CommandIndexA + 1));
    CommandInputB = "0";
  }
  DebugLn("Current Command:", 0);
  DebugLn(String(CurrentCommand), 0);
  DebugLn("Command Inputs:", 0);
  DebugLn(String(CommandInputA), 0);
  DebugLn(String(CommandInputB), 0);                             //Debug output
  if (CurrentCommand == "Brake" || CurrentCommand == "brake") {  //Code to decide which command has been inputted
    Brake();
  } else if (CurrentCommand == "Forwards" || CurrentCommand == "forwards" || CurrentCommand == "Forward" || CurrentCommand == "forward") {
    CommandIntA = CommandInputA.toFloat();  //Needed as by default the inputs are strings
    CommandIntB = CommandInputB.toFloat();
    DebugLn(String("Command Int:"), 0);
    DebugLn(String(CommandIntA), 0);
    DebugLn(String(CommandIntB), 0);
    Forwards(CommandIntA, CommandIntB);
    //Forwards(0.2, 0.4);
  } else if (CurrentCommand == "Start" || CurrentCommand == "start") {
    Start = 1;
    DebugLn("Starting", 0);
  } else if (CurrentCommand == "Stop" || CurrentCommand == "stop") {
    Start = 0;
    DebugLn("Stopping", 0);
  } else if (CurrentCommand == "EnableDebug" || CurrentCommand == "enabledebug" || CurrentCommand == "Enabledebug" || CurrentCommand == "enableDebug") {
    DebugEnabled = 1;
    Serial.print("Debug Enabled set to ");
    Serial.println(DebugEnabled);
  } else if (CurrentCommand == "DisableDebug" || CurrentCommand == "disabledebug" || CurrentCommand == "Disabledebug" || CurrentCommand == "disableDebug") {
    DebugEnabled = 0;
    Serial.print("Debug Enabled set to ");
    Serial.println(DebugEnabled);
  } else if (CurrentCommand == "EnableVerbose" || CurrentCommand == "enableverbose" || CurrentCommand == "Enableverbose" || CurrentCommand == "enableVerbose") {
    VerboseEnabled = 1;
    Serial.print("Verbose Enabled set to ");
    Serial.println(VerboseEnabled);
  } else if (CurrentCommand == "DisableVerbose" || CurrentCommand == "disableverbose" || CurrentCommand == "Disableverbose" || CurrentCommand == "disableVerbose") {
    VerboseEnabled = 0;
    Serial.print("Verbose Enabled set to ");
    Serial.println(VerboseEnabled);
  } else if (CurrentCommand == "EnableBluetooth" || CurrentCommand == "enablebluetooth" || CurrentCommand == "Enablebluetooth" || CurrentCommand == "enableBluetooth") {
    BluetoothEnabled = 1;
    Debug("Bluetooth Enabled set to: ", 0);
    DebugLn(String(BluetoothEnabled), 0);
  } else if (CurrentCommand == "DisableBluetooth" || CurrentCommand == "disablebluetooth" || CurrentCommand == "Disablebluetooth" || CurrentCommand == "disableBluetooth") {
    BluetoothEnabled = 0;
    Debug("Bluetooth Enabled set to: ", 0);
    DebugLn(String(BluetoothEnabled), 0);
  } else if (CurrentCommand == "EnableSerial" || CurrentCommand == "enableserial" || CurrentCommand == "Enableserial" || CurrentCommand == "enableSerial") {
    SerialEnabled = 1;
    Debug("Serial Enabled set to: ", 0);
    DebugLn(String(SerialEnabled), 0);
  } else if (CurrentCommand == "DisableSerial" || CurrentCommand == "disableserial" || CurrentCommand == "Disableserial" || CurrentCommand == "disableSerial") {
    SerialEnabled = 0;
    Debug("Serial Enabled set to: ", 0);
    DebugLn(String(SerialEnabled), 0);
  } else if (CurrentCommand == "EditLoopDelay" || CurrentCommand == "editloopdelay" || CurrentCommand == "Editloopdelay" || CurrentCommand == "editLoopdelay" || CurrentCommand == "editloopDelay" || CurrentCommand == "EditloopDelay" || CurrentCommand == "editLoopDelay" || CurrentCommand == "EditLoopdelay") {
    CommandIntA = CommandInputA.toInt();
    LoopDelay == CommandIntA;
    Debug("Loop Delay Enabled set to: ", 0);
    DebugLn(String(LoopDelay), 0);
  } else if (CurrentCommand == "ResetEncoder" || CurrentCommand == "resetencoder" || CurrentCommand == "resetEncoder" || CurrentCommand == "Resetencoder") {
    DebugLn("Resetting Encoders", 0);
    ResetEncoder();
  } else if (CurrentCommand == "SolveMaze" || CurrentCommand == "solvemaze" || CurrentCommand == "Solvemaze" || CurrentCommand == "solveMaze") {
    DebugLn("Running Dijkstra's Algorithm", 0);
    SolveMaze();
  } else if (CurrentCommand == "FollowLeft" || CurrentCommand == "followleft" || CurrentCommand == "Followleft" || CurrentCommand == "followLeft") {
    DebugLn("Following Left", 0);
    FollowLeft();
  } else if (CurrentCommand == "TestTurn" || CurrentCommand == "testturn" || CurrentCommand == "testTurn" || CurrentCommand == "Testturn") {
  
    TurnLeft();
  } else if (CurrentCommand == "TestMove" || CurrentCommand == "testmove" || CurrentCommand == "testMove" || CurrentCommand == "Testmove") {
    CommandIntA = CommandInputA.toInt();
    CountsPerRotation = CommandIntA;
    PrintMessageLn("Moving Fowards");
    Forwards(0.4, 0.4);
    Encoder(40);
    UpdateLocation();
    Brake();
  } else {
    PrintMessageLn("Unknown Command, did you remember the colons?");
    delay(1000);
  }
}

void PrintMessage(String Message) {  //Used to send outputs over both serial and IR, used to make other sections cleaner
  if (SerialEnabled == 1) {
    Serial.print(Message);
  }
  if (BluetoothEnabled == 1) {  //Doesnt print bluetooth message, only adds to buffer. Will be printed once the PrintMessageLn() command is used. Is due to library problems
    uint8_t buf[Message.length()];
    memcpy(buf, Message.c_str(), Message.length());
    SerialBT.write(buf, Message.length());
  }
}

void PrintMessageLn(String MessageLn) {  //Used to send outputs over both serial and IR, used to make other sections cleaner
  if (SerialEnabled == 1) {
    Serial.println(MessageLn);
  }
  if (BluetoothEnabled == 1) {  //Assembles the message, writes it to buffer then sends buffer
    uint8_t buf[MessageLn.length()];
    memcpy(buf, MessageLn.c_str(), MessageLn.length());
    SerialBT.write(buf, MessageLn.length());
    SerialBT.println();
  }
}

void Debug(String Message, int Verbose) {  //Used to send outputs over both serial and IR, used to make other sections cleaner, separated from PrintMessageLn to minimise interaction with debug
  if (DebugEnabled == 1) {
    if (Verbose == 1) {
      if (VerboseEnabled == 1) {
        if (SerialEnabled == 1) {
          Serial.print(Message);
        }
        if (BluetoothEnabled == 1) {  //Assembles the message, writes it to buffer then sends buffer
          uint8_t buf[Message.length()];
          memcpy(buf, Message.c_str(), Message.length());
          SerialBT.write(buf, Message.length());
        }
      }
    } else {
      if (SerialEnabled == 1) {
        Serial.print(Message);
      }
      if (BluetoothEnabled == 1) {  //Doesnt print bluetooth message, only adds to buffer. Will be printed once the PrintMessageLn() command is used. Is due to library problems
        uint8_t buf[Message.length()];
        memcpy(buf, Message.c_str(), Message.length());
        SerialBT.write(buf, Message.length());
      }
    }
  }
}

void DebugLn(String MessageLn, int Verbose) {  //Used to send outputs over both serial and IR, used to make other sections cleaner, separated from PrintMessageLn to minimise interaction with debug
  if (DebugEnabled == 1) {
    if (Verbose == 1) {
      if (VerboseEnabled == 1) {
        if (SerialEnabled == 1) {
          Serial.println(MessageLn);
        }
        if (BluetoothEnabled == 1) {  //Assembles the message, writes it to buffer then sends buffer
          uint8_t buf[MessageLn.length()];
          memcpy(buf, MessageLn.c_str(), MessageLn.length());
          SerialBT.write(buf, MessageLn.length());
          SerialBT.println();
        }
      }
    } else {
      if (SerialEnabled == 1) {
        Serial.println(MessageLn);
      }
      if (BluetoothEnabled == 1) {  //Assembles the message, writes it to buffer then sends buffer
        uint8_t buf[MessageLn.length()];
        memcpy(buf, MessageLn.c_str(), MessageLn.length());
        SerialBT.write(buf, MessageLn.length());
        SerialBT.println();
      }
    }
  }
}

void InitializeMaze() {  //Sets the maze as all empty spaces

  for (int i = 0; i < N1; i++) {
    for (int j = 0; j < N2; j++) {
      maze[i][j] = 0;  // Initialize all maze cells as unknown space
    }
  }
}

void PrintMaze() {  //Outputs the maze
  for (int i = 0; i < N1; i++) {
    for (int j = 0; j < N2; j++) {
      if (maze[i][j] == 2) {
        PrintMessage("#");  // Wall
      } else if (maze[i][j] == 1) {
        PrintMessage(".");  // Free space
      } else if (maze[i][j] == 0) {
        PrintMessage("?");  // Unknown space
      } else if (maze[i][j] == 3) {
        PrintMessage("X");  // Robot's 5x5 block representation
      }
    }
    PrintMessageLn("");
  }
  PrintMessageLn("");
}

void SolveMaze() {  // Run Dijkstra to solve the maze and get the path

  Dijkstra(startRow, startCol, endRow, endCol, path);
  delay(1000);
  Serial.println("Begin Navigation");
  delay(1000);

  // Debug: Print the path found by Dijkstra
  Serial.println("Path found by Dijkstra:");
  for (auto& step : path) {
    Serial.print("(");
    Serial.print(step.first);
    Serial.print(", ");
    Serial.print(step.second);
    Serial.println(")");
  }

  // Robot's movement simulation
  int currentIdx = 0;
  while (currentIdx < path.size()) {
    Serial.println("------------------------------------------------------------------------------------------------------------------------------------------");
    DijkstraMove(path, currentIdx);  // Move the robot to the next position in the path
  }
}

bool IsValidMove(int row, int col) {  // Check if the robot's 5x5 grid can fit within the maze bounds

  // Check that the robot's 5x5 grid does not go out of bounds and isn't a wall
  if (row < 0 || row + 4 >= N1 || col < 0 || col + 4 >= N2) {
    return false;  // Robot's 5x5 grid exceeds maze boundaries
  }

  // Check that all parts of the robot (5x5 area) are free space (not walls)
  for (int r = row; r < row + 5; r++) {
    for (int c = col; c < col + 5; c++) {
      if (maze[r][c] == 2) {  // If its a wall
        return false;
      }
    }
  }

  return true;  // Valid move :)
}

bool IsAtDestination(int row, int col, int endRow, int endCol) {  // Check if the robot's 5x5 grid has reached the destination

  // The robot's 5x5 grid should cover the destination (endRow, endCol), it will be considered at destination if its bottom-right corner
  // (row + 4, col + 4) matches the destination, fixes for larger area
  if (row + 4 == endRow && col + 4 == endCol) {
    return true;  // The robot's bottom-right corner is at the destination
  }
  return false;
}

void Dijkstra(int startRow, int startCol, int endRow, int endCol, std::vector<std::pair<int, int>>& path) {  //Dijkstra's Algorithm implementation
  std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;

  int dist[N1][N2];  // Distance array, initialized to INF

  for (int i = 0; i < N1; i++) {
    for (int j = 0; j < N2; j++) {
      dist[i][j] = INF;
    }
  }
  dist[startRow][startCol] = 0;  // Starting point

  pq.push({ startRow, startCol, 0 });  // Start with the source cell


  Debug("Start position: ", 0);  // Print the start and end coordinates to ensure they are correct

  Debug(String(startRow), 0);
  Debug(", ", 0);
  DebugLn(String(startCol), 0);
  Debug("End position: ", 0);
  Debug(String(endRow), 0);
  Debug(", ", 0);
  DebugLn(String(endCol), 0);

  while (!pq.empty()) {
    Cell curr = pq.top();
    pq.pop();

    int row = curr.row;
    int col = curr.col;
    int currDist = dist[row][col];

    // Print the current cell being processed
    Debug("Processing cell: ", 1);
    Debug(String(row), 1);
    Debug(", ", 1);
    DebugLn(String(col), 1);

    // Check if the robot's 5x5 grid is at the destination
    if (IsAtDestination(row, col, endRow, endCol)) {
      // Print when destination is found
      Debug("Found destination: ", 0);
      Debug(String(row), 0);
      Debug(", ", 0);
      DebugLn(String(col), 0);

      path.push_back({ row, col });

      // Reconstruct the path by backtracking from the destination
      while (row != startRow || col != startCol) {
        for (int i = 0; i < 4; i++) {
          int newRow = row + dRow[i];
          int newCol = col + dCol[i];
          // Ensure that we are not going out of bounds and the distance is correct
          if (newRow >= 0 && newRow < N1 && newCol >= 0 && newCol < N2 && dist[newRow][newCol] == dist[row][col] - 1) {
            path.push_back({ newRow, newCol });
            row = newRow;
            col = newCol;
            break;
          }
        }
      }

      // Reverse the path so it goes from start to end
      std::reverse(path.begin(), path.end());

      // Print the full path from start to end
      Serial.println("Path from start to end:");
      for (size_t i = 0; i < path.size(); i++) {
        PrintMessage("Step ");
        PrintMessage(String(i));
        PrintMessage(": ");
        PrintMessage(String(path[i].first));
        PrintMessage(", ");
        PrintMessageLn(String(path[i].second));
      }

      return;
    }

    // Explore neighbors (up, down, left, right)
    for (int i = 0; i < 4; i++) {
      int newRow = row + dRow[i];
      int newCol = col + dCol[i];

      // Only process the neighbor if it's a valid move
      if (IsValidMove(newRow, newCol)) {
        int newDist = currDist + 1;  // Each move costs 1 step

        // If a shorter path is found, update the distance and push to the queue
        if (newDist < dist[newRow][newCol]) {
          dist[newRow][newCol] = newDist;
          pq.push({ newRow, newCol, newDist });
        }
      }
    }
  }

  // If no path is found
  Serial.println("No path found!");
}

void DijkstraMove(std::vector<std::pair<int, int>>& path, int& currentIdx) {  //Movement for Dijkstra's Algorithm
  if (currentIdx < path.size() - 1) {
    int currentRow = path[currentIdx].first;
    int currentCol = path[currentIdx].second;
    int nextRow = path[currentIdx + 1].first;
    int nextCol = path[currentIdx + 1].second;

    // Determine the direction to move
    if (nextRow == currentRow - 1 && nextCol == currentCol) {
      RotateTo(NORTH);  // Move up
    } else if (nextRow == currentRow + 1 && nextCol == currentCol) {
      RotateTo(SOUTH);  // Move down
    } else if (nextRow == currentRow && nextCol == currentCol + 1) {
      RotateTo(EAST);  // Move right
    } else if (nextRow == currentRow && nextCol == currentCol - 1) {
      RotateTo(WEST);  // Move left
    }

    MoveSpace();  // Move to the next space

    // Update the maze to show robot's position as a 5x5 block
    for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 5; j++) {
        int robotRow = nextRow + i;
        int robotCol = nextCol + j;
        if (robotRow < N1 && robotCol < N2) {
          maze[robotRow][robotCol] = 3;  // Robot's 5x5 block
        }
      }
    }

    PrintMaze();
    delay(500);  // Slow down the movement for visibility

    // Clear the previous position of the robot (optional, to make movement clearer)
    for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 5; j++) {
        int robotRow = currentRow + i;
        int robotCol = currentCol + j;
        if (robotRow < N1 && robotCol < N2 && maze[robotRow][robotCol] == 3) {
          maze[robotRow][robotCol] = 1;  // Reset the position back to free space
        }
      }
    }

    currentIdx++;  // Move to the next step in the path

    // Print the current position
    Debug("Robot moved to: ", 0);
    Debug(String(nextRow), 0);
    Debug(", ", 0);
    DebugLn(String(nextCol), 0);
  }
}

void ResetEncoder() {  //Sets the encoders to zero, used later
  EncoderX.numberRotation = 0;
  EncoderY.numberRotation = 0;
}

void PollSensor() {  //Checks the sensors and adds the values to the IR storage array
  /*
  CheckUltrasonicSensor0();
  CheckUltrasonicSensor1();
  CheckUltrasonicSensor2();
  */
  IrLoop();
}

void MoveSpace() {  //Not set up with correct values, needs to be tested, currently just set up to go based on a timer, ideally would use an encoder (code for which is below)
  PrintMessageLn("Moving Fowards");
  Forwards(0.4, 0.4);
  Encoder(CellSize);
  Brake();
  UpdateLocation();
}

void TurnLeft() {  //Turns left
  Turning = true;
  PrintMessageLn("Turning Left");
  Forwards(-0.4, 0.4);
  Encoder(WheelGap);
  Brake();
  MovementList[MovementCounter] = 'B';
  MovementCounter = MovementCounter + 1;
  currentDirection = static_cast<Direction>((currentDirection + 3) % 4);  // Decrementing direction and wrapping, subtracting didnt work, but adding 3 and wrapping works
  Turning = false;
}

void TurnRight() {  //Turns right
  Turning = true;
  PrintMessageLn("Turning Right");
  Forwards(0.4, -0.4);
  Encoder(WheelGap);
  Brake();
  MovementList[MovementCounter] = 'C';
  MovementCounter = MovementCounter + 1;
  currentDirection = static_cast<Direction>((currentDirection + 1) % 4);  // Incrementing direction and wrapping, wrapping fixes directions "after" west
  Turning = false;
}

void TurnBack() {  //Turns left twice
  TurnLeft();
  TurnLeft();
}

void RotateTo(Direction targetDirection) {  //Rotates to a target direction, using the enum directions declared at the start
  while (currentDirection != targetDirection) {
    if ((currentDirection + 1) % 4 == targetDirection) {
      TurnRight();
    } else {
      TurnLeft();
    }
  }
}

void Forwards(float x, float y) {  //Command to run both motors based on the input, can be called from terminal (Forwards:x:y)
  PrintMessage("Moving Forwards, Motor speed values set to x = ");
  PrintMessage(String(x));
  PrintMessage(" y = ");
  PrintMessageLn(String(y));
  motors.drive(x, y);
}

void Brake() {  //Brake functions for both motors, can be called from terminal (Brake::)
  PrintMessageLn("Braking");
  motors.brake();
}

void CheckUltrasonicSensor0() {  //Checks Front Ultrasonic Sensor
  // Clears the trigPin
  digitalWrite(trigPin0, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin0, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin0, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin0, HIGH);
  // Calculating the distance
  distance = duration * 0.343 / 2;
  // Prints the distance on the Serial Monitor
  PrintMessage("Front ");
  PrintMessageLn(String(distance));
  DistanceValues[0] = distance;
}

void CheckUltrasonicSensor1() {  //Checks Left Ultrasonic Sensor
  // Clears the trigPin
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin1, HIGH);
  // Calculating the distance
  distance = duration * 0.343 / 2;
  // Prints the distance on the Serial Monitor
  PrintMessage("Left ");
  PrintMessageLn(String(distance));
  DistanceValues[1] = distance;
}

void CheckUltrasonicSensor2() {  //Checks Right Ultrasonic Sensor
  // Clears the trigPin
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin2, HIGH);
  // Calculating the distance
  distance = duration * 0.343 / 2;
  // Prints the distance on the Serial Monitor
  PrintMessage("Right ");
  PrintMessageLn(String(distance));
  DistanceValues[2] = distance;
}

void FindWall() {  //Checks wall locations
  IsWallFront();
  IsWallLeft();
  IsWallRight();
}

void IsWallFront() {  //Checks for wall infront
  if (DistanceValues[0] < UltrasoundFrontThreshold) {
    WallFront = 1;
    PrintMessageLn("Wall Found Front");
  } else {
    WallFront = 0;
  }
  if (DistanceValues[0] < DistanceErrorThreshold) {
    PlaceWall(currentDirection, DistanceValues[0]);
  }
}

void IsWallLeft() {  //Checks for wall to the left
  if (DistanceValues[1] < UltrasoundLeftThreshold) {
    WallLeft = 1;
    PrintMessageLn("Wall Found Left");
  } else {
    WallLeft = 0;
  }
  if (DistanceValues[1] < DistanceErrorThreshold) {
    PlaceWall(static_cast<Direction>((currentDirection + 3) % 4), DistanceValues[2]);
  }
}

void IsWallRight() {  //Checks for wall to the right
  if (DistanceValues[0] < UltrasoundRightThreshold) {
    WallRight = 1;
    PrintMessageLn("Wall Found Right");
  } else {
    WallRight = 0;
  }
  if (DistanceValues[0] < DistanceErrorThreshold) {
    PlaceWall(static_cast<Direction>((currentDirection + 1) % 4), DistanceValues[1]);
  }
}

void PlaceWall(Direction wallDirection, int wallDistance) {
  int wallCellDistance = wallDistance / CellSize;

  if (wallDirection == NORTH) {
    for (int i = 1; i <= wallCellDistance; i++) {
      if (CurrentX + i > 0) {
        if (i == wallCellDistance) {
          maze[CurrentX + i][CurrentY] = 2;  // Wall
          Serial.print("Wall placed at: ");
          Serial.print(CurrentX + i);
          Serial.print(", ");
          Serial.println(CurrentY);
        } else {
          maze[CurrentX + i][CurrentY] = 1;  // Free space
          /*
          Serial.print("Free space at: ");
          Serial.print(CurrentX + i);
          Serial.print(", ");
          Serial.println(CurrentY);
          */
        }
      }
    }
  }

  if (wallDirection == SOUTH) {
    for (int i = 1; i <= wallCellDistance; i++) {
      if (CurrentX - i > 0) {
        if (i == wallCellDistance) {
          maze[CurrentX - i][CurrentY] = 2;  // Wall
          Serial.print("Wall placed at: ");
          Serial.print(CurrentX - i);
          Serial.print(", ");
          Serial.println(CurrentY);
        } else {
          maze[CurrentX - i][CurrentY] = 1;  // Free space
          Serial.print("Free space at: ");
          Serial.print(CurrentX - i);
          Serial.print(", ");
          Serial.println(CurrentY);
        }
      }
    }
  }

  if (wallDirection == EAST) {
    for (int i = 1; i <= wallCellDistance; i++) {
      if (CurrentY + i > 0) {
        if (i == wallCellDistance) {
          maze[CurrentX][CurrentY + i] = 2;  // Wall
          Serial.print("Wall placed at: ");
          Serial.print(CurrentX);
          Serial.print(", ");
          Serial.println(CurrentY + i);
        } else {
          maze[CurrentX][CurrentY + i] = 1;  // Free space
          Serial.print("Free space at: ");
          Serial.print(CurrentX);
          Serial.print(", ");
          Serial.println(CurrentY + i);
        }
      }
    }
  }

  if (wallDirection == WEST) {
    for (int i = 1; i <= wallCellDistance; i++) {
      if (CurrentY - i > 0) {
        if (i == wallCellDistance) {
          maze[CurrentX][CurrentY - i] = 2;  // Wall
          Serial.print("Wall placed at: ");
          Serial.print(CurrentX);
          Serial.print(", ");
          Serial.println(CurrentY - i);
        } else {
          maze[CurrentX][CurrentY - i] = 1;  // Free space
          Serial.print("Free space at: ");
          Serial.print(CurrentX);
          Serial.print(", ");
          Serial.println(CurrentY - i);
        }
      }
    }
  }
}



void UpdateLocation() {  //Updates current position
  DebugLn(String(currentDirection), 0);
  if (currentDirection == NORTH) {
    if (CurrentX >= 0) {
      CurrentX = CurrentX + 1;
    }
  }

  if (currentDirection == EAST) {
    if (CurrentY >= 0) {
      CurrentY = CurrentY + 1;
    }
  }

  if (currentDirection == SOUTH) {
    if (CurrentX < N1) {
      CurrentX = CurrentX - 1;
    }
  }

  if (currentDirection == WEST) {
    if (CurrentY < N2) {
      CurrentY = CurrentY - 1;
    }
  }

  maze[CurrentX][CurrentY] = 1;
}

void Encoder(int TargetDistance) {  //Similar to a wait function, delays the function until the distance is met

  ResetDistance();
  ResetEncoder();

  while (DistanceTravelled < TargetDistance) {
    // Update distance traveled
    DistanceTravelled = int(((LeftCounter + RightCounter) / 2) * WheelCircumference / CountsPerRotation);



    // Check for encoder updates
    noInterrupts();  // Temporarily disable interrupts to safely read shared data
    uint32_t leftCount = EncoderX.numberRotation;
    uint32_t rightCount = EncoderY.numberRotation;
    interrupts();  // Re-enable interrupts

    // Update counters
    LeftCounter = leftCount;
    RightCounter = rightCount;


    // Allow other tasks to run
    vTaskDelay(10 / portTICK_PERIOD_MS);  // FreeRTOS delay for multitasking
  }
  Brake();

  // Print debug information
  Debug("LeftCounter: ", 1);
  Debug(String(LeftCounter), 1);
  Debug("RightCounter: ", 1);
  Debug(String(RightCounter), 1);
  Debug("DistanceTravelled", 1);
  DebugLn(String(DistanceTravelled), 1);

  // Calculate encoder difference
  EncoderDifference = LeftCounter - RightCounter;
  Debug("Difference between encoders:", 0);
  DebugLn(String(EncoderDifference), 0);


  PollSensor();
  FindWall();
  ResetDistance();
}

void ResetDistance() {  //Resets the encoder distances
  DistanceTravelled = 0;
  LeftCounter = 0;
  RightCounter = 0;
}

void FollowLeft() {  //Solve using follow left wall
  while (IsAtDestination(CurrentX, CurrentY, endRow, endCol) == false) {
    FindWall();
    if (WallLeft == 0) {
      TurnLeft();
      MoveSpace();
    } else {
      if (WallFront == 0) {
        MoveSpace();
      } else if (WallRight == 0) {
        TurnRight();
        MoveSpace();
      } else {
        TurnBack();
        MoveSpace();
      }
    }
  }
  PrintMessageLn("Maze Solved :)");
}

void PrintSensors() {  //Prints the values from the sensors, for debug
  PollSensor();
  FindWall();
  PrintMessage("Sensor 1: ");
  PrintMessage(String(DistanceValues[0]));
  PrintMessage("Sensor 2: ");
  PrintMessage(String(DistanceValues[1]));
  PrintMessage("Sensor 3: ");
  PrintMessageLn(String(DistanceValues[2]));
}

void IrLoop() {
  for (uint8_t channel = 0; channel <= 3; channel++) {
    resetMultiplexer();  // Reset multiplexer to avoid residual channel interference
    selectChannel((channel));
    delay(100);

    // Debugging: Scan for active devices
    // scanI2C();

    // Read sensor data
    int distance = readDistance() / 100;
    if (distance != -1) {
      /*Serial.print("Distance at channel ");
      Serial.print(channel);
      Serial.print(": ");
      Serial.print(distance);
      Serial.println(" mm");*/
      DistanceValues[(channel)] = distance;
    }
  }
}

void selectChannel(uint8_t channel) {
  if (channel > 7) {
    Serial.println("Invalid channel selected!");
    return;
  }
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);  // Select the desired channel
  if (Wire.endTransmission() == 0) {
    Serial.print("Channel ");
    Serial.print(channel);
    Serial.println(" selected.");
  } else {
    Serial.println("Error selecting channel.");
  }
  delay(200);  // Stabilization delay
}

int readDistance() {
  // Write the register address
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x5E);  // Register for distance
  if (Wire.endTransmission() != 0) {
    Serial.println("Error writing to sensor.");
    return -1;
  }

  delay(10);  // Allow sensor to prepare data

  // Request 2 bytes
  Wire.requestFrom(SENSOR_ADDR, 2);
  if (Wire.available() == 2) {
    int msb = Wire.read();
    int lsb = Wire.read();
    Serial.print("Raw Data: MSB=0x");
    Serial.print(msb, HEX);
    Serial.print(", LSB=0x");
    Serial.println(lsb, HEX);

    return (msb << 8) | lsb;
  } else {
    Serial.println("Error reading sensor data.");
    return -1;
  }
}

void scanI2C() {
  Serial.println("Scanning I2C bus...");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Scan complete.");
}

void resetMultiplexer() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0x00);  // Disable all channels
  Wire.endTransmission();
  delay(100);  // Allow time to reset
}
