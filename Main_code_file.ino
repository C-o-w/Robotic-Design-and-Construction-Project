  // Main change to take note of, using PrintMessage() and PrintMessageLn() instead of print to allow it to switch between bluetooth and serial depending which is enabled with variables (line 178-199)
  // Debug and DebugLn also included, 0 for regular debug, 1 for verbose debug

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

  //Creation of the bluetooth object
  BluetoothSerial SerialBT;

  // Control Variables
  int Start = 1;
  int LoopDelay = 1;

  // defines pins numbers
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

  int UltrasoundFrontThreshold = 5;
  int UltrasoundRightThreshold = 10;
  int UltrasoundLeftThreshold = 10;

  //Array to store the IR values
  int UltrasoundValues[3];

  //Motor Setup
  #define MotorAPWM 13  //D3
  #define MotorA1 14    //D4
  #define MotorA2 12    //D5
  #define MotorBPWM 33  //D6
  #define MotorB1 26    //D7
  #define MotorB2 25    //D8
  #define STBY 27       //D9

  //Creation of motor controller objects for each motor
  /*
  Tb6612fng motor1(STBY, MotorA1, MotorA2, MotorAPWM);
  Tb6612fng motor2(STBY, MotorB1, MotorB2, MotorBPWM);
  */
  Tb6612fng motors(STBY, MotorA1, MotorA2, MotorAPWM, MotorB1, MotorB2, MotorBPWM);

  //Variables for the rotary encoders
  #define LeftOutputA 35
  #define LeftOutputB 34
  #define RightOutputA 15
  #define RightOutputB 2

  int LeftCounter = 0;
  int LeftState;
  int LeftLastState;

  int RightCounter = 0;
  int RightState;
  int RightLastState;

  int DistanceTravelled = 0;
  int DistanceOffsetX = 0;
  int DistanceOffsetY = 0;
  String MoveAxis = "X";

  int WheelDiameter = 1;
  float WheelCircumference = 0;  //Calculated in setup
  int CountsPerRotation = 600;
  int EncoderDifference;

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

  int MovementCounter = 0;
  char MovementList[] = {};  //Forwards = A, TurnLeft = B, TurnRight = C, Back = D

  #define N1 40       // Size of the maze (20x20) ROW
  #define N2 40       //                          COL
  #define INF 999999  // Infinity, representing unvisited cells
  int CellSize = 4; //Size of one cell in MM

  // Directions for moving in the maze (up, down, left, right)
  int dRow[] = { -1, 1, 0, 0 };  // Row direction for up and down
  int dCol[] = { 0, 0, -1, 1 };  // Column direction for left and right

  // Maze: 0 for unknown, 1 for free space, 2 for wall, temporoary simulated map, in future fill with 0s as base and fill information as found
  int maze[N1][N2] = {

    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1 }
  };


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
    const uint8_t PIN;
    uint32_t numberRotation;
    bool triggered;
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

  void RotateTo(Direction targetDirection); // Running into compilation error, declaring a function prototype fixed them

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

    motors.begin();                //Begins the motor object
    Serial.begin(9600);            //Serial began at a baud rate of 9600, baud rate doesnt matter particularly but must match on the terminal
    SerialBT.begin("MicroMouse");  //Bluetooth device name
    PrintMessageLn("Setup Complete");
    delay(100);
  }

  void loop() {
    CheckCommand();

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

  //CHECKCOMMAND AND READCOMMAND ARE COMPLETELY UNNECESSARY TO THE FUNCTIONALITY OF THE DEVICE, PURELY DEBUG I DONT KNOW HOW IT WORKS AND IM TOO AFRAID TO MESS WITH IT
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

  void Debug(String Message, int Verbose) {  //Used to send outputs over both serial and IR, used to make other sections cleaner
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

  void DebugLn(String MessageLn, int Verbose) {  //Used to send outputs over both serial and IR, used to make other sections cleaner
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

  void InitializeMaze() {

    for (int i = 0; i < N1; i++) {
      for (int j = 0; j < N2; j++) {
        maze[i][j] = 1;  // Initialize all maze cells as free space
      }
    }
  }

  void PrintMaze() {
    for (int i = 0; i < N1; i++) {
      for (int j = 0; j < N2; j++) {
        if (maze[i][j] == 2) {
          Serial.print("#");  // Wall
        } else if (maze[i][j] == 1) {
          Serial.print(".");  // Free space
        } else if (maze[i][j] == 0) {
          Serial.print("?");  // Unknown space
        } else if (maze[i][j] == 3) {
          Serial.print("X");  // Robot's 5x5 block representation
        }
      }
      Serial.println();
    }
    Serial.println();
  }

  void SolveMaze() {
    // Run Dijkstra to solve the maze and get the path
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

  bool isValidMove(int row, int col) {  // Check if the robot's 5x5 grid can fit within the maze bounds

    // Check that the robot's 5x5 grid does not go out of bounds and isn't a wall
    if (row < 0 || row + 4 >= N1 || col < 0 || col + 4 >= N2) {
      return false;  // Robot's 5x5 grid exceeds maze boundaries
    }

    // Check that all parts of the robot (5x5 area) are free space (not walls)
    for (int r = row; r < row + 5; r++) {
      for (int c = col; c < col + 5; c++) {
        if (maze[r][c] == 2) {  // Wall
          return false;
        }
      }
    }

    return true;  // Valid move
  }

  bool isAtDestination(int row, int col, int endRow, int endCol) {  // Check if the robot's 5x5 grid has reached the destination

    // The robot's 5x5 grid should cover the destination (endRow, endCol)
    // The robot will be considered at the destination if its bottom-right corner
    // (row + 4, col + 4) matches the destination.
    if (row + 4 == endRow && col + 4 == endCol) {
      return true;  // The robot's bottom-right corner is at the destination
    }
    return false;
  }

  void Dijkstra(int startRow, int startCol, int endRow, int endCol, std::vector<std::pair<int, int>>& path) {
    // Priority queue for Dijkstra's algorithm
    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;

    // Distance array, initialized to INF
    int dist[N1][N2];
    for (int i = 0; i < N1; i++) {
      for (int j = 0; j < N2; j++) {
        dist[i][j] = INF;
      }
    }
    dist[startRow][startCol] = 0;  // Starting point

    // Start with the source cell
    pq.push({ startRow, startCol, 0 });

    // Debug: Print the start and end coordinates to ensure they are correct
    Serial.print("Start position: ");
    Serial.print(startRow);
    Serial.print(", ");
    Serial.println(startCol);
    Serial.print("End position: ");
    Serial.print(endRow);
    Serial.print(", ");
    Serial.println(endCol);

    while (!pq.empty()) {
      Cell curr = pq.top();
      pq.pop();

      int row = curr.row;
      int col = curr.col;
      int currDist = dist[row][col];

      // Debug: Print the current cell being processed
      Serial.print("Processing cell: ");
      Serial.print(row);
      Serial.print(", ");
      Serial.println(col);

      // Check if the robot's 5x5 grid is at the destination
      if (isAtDestination(row, col, endRow, endCol)) {
        // Debug: Print when destination is found
        Serial.print("Found destination: ");
        Serial.print(row);
        Serial.print(", ");
        Serial.println(col);

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
          Serial.print("Step ");
          Serial.print(i);
          Serial.print(": ");
          Serial.print(path[i].first);
          Serial.print(", ");
          Serial.println(path[i].second);
        }

        return;
      }

      // Explore neighbors (up, down, left, right)
      for (int i = 0; i < 4; i++) {
        int newRow = row + dRow[i];
        int newCol = col + dCol[i];

        // Only process the neighbor if it's a valid move
        if (isValidMove(newRow, newCol)) {
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

  void DijkstraMove(std::vector<std::pair<int, int>>& path, int& currentIdx) {
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

      // Debug: Print the current position
      Serial.print("Robot moved to: ");
      Serial.print(nextRow);
      Serial.print(", ");
      Serial.println(nextCol);
    }
  }

  void ResetEncoder() {
    EncoderX.numberRotation = 0;
    EncoderY.numberRotation = 0;
  }

  void ControlLoop() {  //Loop to control movement
  }

  void PollSensor() {  //Checks the sensors and adds the values to the IR storage array
    CheckUltrasonicSensor1();
    CheckUltrasonicSensor2();
    //CheckSensor3();
    LeftLastState = digitalRead(LeftOutputA);
    RightLastState = digitalRead(RightOutputA);
  }

  void MoveSpace() {  //Not set up with correct values, needs to be tested, currently just set up to go based on a timer, ideally would use an encoder (code for which is below)
    PrintMessageLn("Moving Fowards");
    Forwards(0.2, -0.195);
    Encoder(CellSize);
    Brake();
  }

  void TurnLeft() {

    PrintMessageLn("Turning Left");
    motors.drive(-0.2, -0.2, 490);
    delay(100);
    Brake();
    MovementList[MovementCounter] = 'B';
    MovementCounter = MovementCounter + 1;
    currentDirection = static_cast<Direction>((currentDirection + 3) % 4);
  }

  void TurnRight() {

    PrintMessageLn("Turning Right");
    motors.drive(0.20, 0.2, 490);
    delay(100);
    Brake();
    MovementList[MovementCounter] = 'C';
    MovementCounter = MovementCounter + 1;
    currentDirection = static_cast<Direction>((currentDirection + 3) % 4);
  }

  void TurnBack() {
    TurnLeft();
    TurnLeft();
  }

  void RotateTo(Direction targetDirection) {
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
    motors.drive(x, y, 10, false);
  }

  void Brake() {  //Brake functions for both motors, can be called from terminal (Brake::)
    PrintMessageLn("Braking");
    motors.brake();
  }

  void CheckUltrasonicSensor1() {
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
    distance = duration * 0.0343 / 2;
    // Prints the distance on the Serial Monitor
    PrintMessage("Front ");
    PrintMessageLn(String(distance));
    UltrasoundValues[0] = distance;
  }

  void CheckUltrasonicSensor2() {
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
    distance = duration * 0.0343 / 2;
    // Prints the distance on the Serial Monitor
    PrintMessage("Right ");
    PrintMessageLn(String(distance));
    UltrasoundValues[1] = distance;
  }

  void IsWallFront() {  //Checks for wall infront
    if (UltrasoundValues[0] < UltrasoundFrontThreshold) {
      WallFront = 1;
      PrintMessageLn("Wall Found Front");
    } else {
      WallFront = 0;
    }
  }

  void IsWallLeft() {  //Checks for wall to the left
    if (UltrasoundValues[2] < UltrasoundLeftThreshold) {
      WallLeft = 1;
      PrintMessageLn("Wall Found Left");
    } else {
      WallLeft = 0;
    }
  }

  void IsWallRight() {  //Checks for wall to the right
    if (UltrasoundValues[1] < UltrasoundRightThreshold) {
      WallRight = 1;
    } else {
      WallRight = 0;
    }
  }

  void Encoder(int TargetDistance) {
    if (MoveAxis == "x") {  //Use an offset to adjust for overshooting the encoder distance
      DistanceTravelled = 0 - DistanceOffsetX;
    } else if (MoveAxis == "Y") {
      DistanceTravelled = 0 - DistanceOffsetY;
    } else {
      DistanceTravelled = 0;
    };

    LeftCounter = 0;
    RightCounter = 0;

    while (DistanceTravelled < TargetDistance) {
      Debug("Difference between encoders:", 0);
      EncoderDifference = (LeftCounter - RightCounter);
      DebugLn(String(EncoderDifference), 0);
      DistanceTravelled = int(((LeftCounter + RightCounter) / 2) * WheelCircumference / CountsPerRotation);  //Update distance travelled based on the average rotations per encoder
    };
  }
