#include <NewPing.h>         // HC-SR04 (Ultrasonic Sensor)
#include <SoftwareSerial.h>  // HC-05 (Bluetooth Module)
#include <StackArray.h>      // LIFO

// MPU-6050 (Gyroscope)
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// ---------------------------------------- GLOBAL VARIABLES ----------------------------------------

const int MoveDelay = 1000;   // time in ms to move for (optimize for traversing distance of each block)
const int StopDelay = 1000;   // time in ms to stop for

const int MaxPingDistance = 200;  // max distance to ping ultrasonic (cm)
const int MaxWallDistance = 25;   // less than this means wall (cm)

// orientation of robot (0 is north) -- increment on right turn, decrement on left turn
int Orient = 0;

// Gyroscope
const int MpuInterrupt = 2;

MPU6050 mpu;

// Bluetooth
const int BluetoothTX = 3;
const int BluetoothRX = 4;

SoftwareSerial Bluetooth(BluetoothTX, BluetoothRX);

// Motors
const int MotorIN1 = 5;
const int MotorIN2 = 6;
const int MotorIN3 = 7;
const int MotorIN4 = 8;
const int MotorEnable = 9;  // common enable for both motors

// Ultrasonic
const int RightEcho = 10;
const int FrontEcho = 11;
const int LeftEcho = 12;
const int Trig = 13;      // common trig for all sensor

NewPing FrontSonar(Trig, FrontEcho, MaxPingDistance);
NewPing RightSonar(Trig, RightEcho, MaxPingDistance);
NewPing LeftSonar(Trig, LeftEcho, MaxPingDistance);

// maze dimensions
const int ROWS = 5;
const int COLS = 5;
const int GRID_ROWS = ROWS * 2 - 1;
const int GRID_COLS = COLS * 2 - 1;

// block value stored as ...0000WSEN
int Grid[GRID_ROWS][GRID_COLS] = {0};    // grid exploration array
int MappedMaze[ROWS][COLS] = {0};        // final maze for transmission

// location of block
struct Point
{
  Point(int row, int col) : r(row), c(col) {}

  int r, c;
};

typedef StackArray<Point> PointStack;

// add all paths to get value of point
enum Path
{
  U = 0, // unmapped
  N = 1, // north
  E = 2, // east
  S = 4, // south
  W = 8, // west
};

// column index of neighbors array
enum Neighbors
{
  dir, row, col
};

// adjacent cells (up, right, down, left)
const int Neighbors[4][3] = 
{
  {N, -1, 0},
  {E, 0, 1},
  {S, 1, 0},
  {W, 0, -1}
};

// for testing
int SampleMaze[ROWS][COLS] = 
{
  {S | E, S | W | E, W | E, S | W | E, W},
  {N, N | S, E, N | S | W, S},
  {S, N | E, S | W | E, N | W | E, N | W},
  {N | E, W | E, N | W | S, S | E, W},
  {E, W | E, N | W | E, N | W | E, W}
};

// ---------------------------------------- MAZE MAPPING ----------------------------------------

// LSB 4 bits cyclic shift right
int CyclicShiftRight(int val)
{
  return val >> 1 | (val & 1) << 3;
}

// LSB 4 bits cyclic shift left
int CyclicShiftLeft(int val)
{
  return (val & 7) << 1 | (val & 8) >> 3;
}

// get value relative to starting orientation of robot
int GetGlobalValue(int val)
{
  Orient %= 4; // 4 turns return to original direction

  if (Orient > 0) // right turn(s)
    for (int i = 0; i < Orient; i++)
      val = CyclicShiftLeft(val);
  else if (Orient < 0)  // left turn(s)
    for (int i = Orient; i < 0; i++)
      val = CyclicShiftRight(val);

  return val;
}

// get value relative to current orientation of robot
int GetLocalValue(int val)
{
  Orient *= -1;
  val = GetGlobalValue(val);
  Orient *= -1;
  return val;
}

// generate global block value by using sensor data
int GetBlockValue(int InitVal)    // init val for wall (behind robot) = U, path = S
{
  // distance values in cm
  int FrontDistance = ReadPing(FrontSonar);
  int RightDistance = ReadPing(RightSonar);
  int LeftDistance = ReadPing(LeftSonar);

  int val = InitVal;   // path through which robot entered current cell

  if (FrontDistance > MaxWallDistance)
    val |= N;

  if (RightDistance > MaxWallDistance)
    val |= E;

  if (LeftDistance > MaxWallDistance)
    val |= W;

  return GetGlobalValue(val);
}

// add one unvisited neighbor to stack
bool AddUnvisitedNeighbor(Point p, PointStack& Stack)
{
  for (int i = 0; i < 4; i++)
  {
    Point adj = Point(p.r + Neighbors[i][row], p.c + Neighbors[i][col]);
    
    if (Grid[p.r][p.c] & Neighbors[i][dir] && !Grid[adj.r][adj.c])  // path to neighbor and not visited
    {
      Stack.push(adj);
      return true;
    }
  }

  return false;
}

// explore and map maze
void MapMaze()
{
  Point prev = Point(GRID_ROWS/2, GRID_COLS/2);
  Grid[prev.r][prev.c] = GetBlockValue(U);
  
  PointStack Stack; // points that are not fully explored
  Stack.push(prev);

  while (!Stack.isEmpty())
  {
    Point curr = Stack.peek();

    MoveToPoint(prev, curr);

    if (!Grid[curr.r][curr.c])  // not visited
      Grid[curr.r][curr.c] = GetBlockValue(S);   // block mapped (visited)

    if (!AddUnvisitedNeighbor(curr, Stack))   // no unexplored neighbors left for curr pt
      Stack.pop();    // remove curr pt from stack

    prev = curr;
  }
  
  SaveMaze();
}

// get first point of Maze from Grid
Point GetFirstPoint()
{
  for (int i = 0; i < GRID_ROWS; i++)
    for (int j = 0; j < GRID_COLS; j++)
      if (Grid[i][j])
        return Point(i, j);
}

// get starting point of robot relative to Maze
Point GetStartPoint()
{
  Point p = GetFirstPoint();
  return Point(GRID_ROWS/2 - p.r, GRID_COLS/2 - p.c); 
}

// update Maze from Grid
void SaveMaze()
{
  Point p = GetFirstPoint();

  for (int i = 0; i < ROWS; i++)
    for (int j = 0; j < COLS; j++)
      MappedMaze[i][j] = Grid[p.r+i][p.c+j];
}

// ---------------------------------------- GYROSCOPE ----------------------------------------

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw = 0.0f;       // stores yaw value used for turning

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() 
{
    mpuInterrupt = true;
}

void InitializeMPU()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  pinMode(MpuInterrupt, INPUT);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) 
  {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(MpuInterrupt), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void UpdateYaw()
{
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    yaw = ypr[0] * 180/M_PI;
    
    if (yaw < 0)  // keep yaw between 0 and 360
      yaw += 360;
  }
}

// ---------------------------------------- ROBOT MOVEMENT ----------------------------------------

int ReadPing(NewPing sonar)
{
  delay(100);
  int distance = sonar.ping_cm();
  
  if (distance == 0)  // no ping received
    distance = MaxPingDistance;
    
  return distance;
}

void Stop()
{
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  delay(StopDelay);
}

void MoveBackward()
{
  digitalWrite(MotorIN3, HIGH);
  digitalWrite(MotorIN4, LOW);
  digitalWrite(MotorIN1, HIGH);
  digitalWrite(MotorIN2, LOW);
  delay(MoveDelay);
}

void MoveForward()
{
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, HIGH);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, HIGH);
  delay(MoveDelay);
}

void Right()
{
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, HIGH);
  digitalWrite(MotorIN1, HIGH);
  digitalWrite(MotorIN2, LOW);
}

void Left()
{
  digitalWrite(MotorIN3, HIGH);
  digitalWrite(MotorIN4, LOW);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, HIGH);
}

int GetHeading()
{
  int heading = Orient * 90;
  heading %= 360; // complete rotation

  if (heading < 0)  // heading must be positive
    heading += 360;

  return heading;
}

void TurnRight90()  // 90 deg right turn
{
  ++Orient;
  int heading = GetHeading();
    
  float err = 2.0f;  // error of +-2 degrees allowed
  
  while (true)
  {
    UpdateYaw();

    if (yaw >= heading - err && yaw <= heading + err)
    {
      Stop();
      return;
    }
    else
    {
      Right();
    }

    delay(10);
  }
}

void TurnLeft90()   // 90 deg left turn
{
  --Orient;
  int heading = GetHeading();
    
  float err = 2.0f;  // error of +-2 degrees allowed
  
  while (true)
  {
    UpdateYaw();

    if (yaw >= heading - err && yaw <= heading + err)
    {
      Stop();
      return;
    }
    else
    {
      Left();
    }

    delay(10);
  }  
}

// get local direction of neighbor from source pt
int GetDirection(Point src, Point dest)
{
  int dr = dest.r - src.r;
  int dc = dest.c - src.c;

  int Dir = 0;

  for (int i = 0; i < 4; i++)
  {
    if (dr == Neighbors[i][row] && dc == Neighbors[i][col])
    {
      Dir = Neighbors[i][dir];
      break;
    }
  }

  return GetLocalValue(Dir);
}

// assumes that dest is a direct neighbor and path to that neighbor exists
// returns false if no path exists
bool MoveToPoint(Point src, Point dest)
{
  if (src.r == dest.r && src.c == dest.c)
    return true;

  switch(GetDirection(src, dest))
  {
    case N:
      MoveForward();
      Stop();
      break;
    case E:
      TurnRight90();
      Stop();
      MoveForward();
      Stop();
      break;
    case S:
      TurnRight90();
      Stop();
      TurnRight90();
      Stop();
      MoveForward();
      Stop();
      break;
    case W:
      TurnLeft90();
      Stop();
      MoveForward();
      Stop();
      break;
    default:
      return false;
  }

  return true;
}

// ---------------------------------------- BLUETOOTH ----------------------------------------

// transmit starting pt of robot and 2D maze array
void TransmitMaze()
{
  while (!Bluetooth.available()) {}   // wait to establish connection

  Point p = GetStartPoint();
  Bluetooth.write(p.r);   // send starting row
  Bluetooth.write(p.c);   // send starting col
  
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++)
      Bluetooth.write(MappedMaze[i][j]);  // send maze block
}

// ---------------------------------------- MAIN FUNCTIONS ----------------------------------------

void setup()
{
  pinMode(MotorIN1, OUTPUT);
  pinMode(MotorIN2, OUTPUT);
  pinMode(MotorIN3, OUTPUT);
  pinMode(MotorIN4, OUTPUT);
  pinMode(MotorEnable, OUTPUT);
  digitalWrite(MotorEnable, HIGH);

  delay(5000);
  InitializeMPU();

  MapMaze();
  Bluetooth.begin(9600);
  TransmitMaze();
}

void loop() {}
