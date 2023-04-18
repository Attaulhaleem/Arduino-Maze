#include <SoftwareSerial.h> // HC-05 (Bluetooth Module)
#include <QueueArray.h>     // FIFO
#include <StackArray.h>     // LIFO

// MPU-6050 (Gyroscope)
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// ---------------------------------------- GLOBAL VARIABLES ----------------------------------------

const int MoveDelay = 1000;  // time in ms to move for (optimize for traversing distance of each block)
const int StopDelay = 1000; // time in ms to stop for

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

// maze dimensions
const int ROWS = 5;
const int COLS = 5;

int Maze[ROWS][COLS] = {0}; // overwritten by data from Ragnar

// location of block
struct Point
{
    Point(int row, int col) : r(row), c(col) {}

    int r, c;
};

Point StartPoint(0, 0); // overwritten by value from Ragnar

// defining maze corners
const Point TopLeftCorner = Point(0, 0);
const Point TopRightCorner = Point(0, COLS - 1);
const Point BottomLeftCorner = Point(ROWS - 1, 0);
const Point BottomRightCorner = Point(ROWS - 1, COLS - 1);

typedef QueueArray<Point> PointQueue;
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

// column index of Neighbors array
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

// ---------------------------------------- PATH FINDING ----------------------------------------

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

// returns false if point out of maze bounds
bool IsValid(Point p)
{
  return p.r >= 0 && p.r < ROWS && p.c >= 0 && p.c < COLS;
}

// trace path from dest to src after filling distance array
PointStack BacktracePath(const int Maze[ROWS][COLS], const Point src, const Point dest, const int Distance[ROWS][COLS])
{
  PointStack Path;
  Path.push(dest);
  
  Point curr = dest;

  while (true)
  {
    if (curr.r == src.r && curr.c == src.c)
      break;

    for (int i = 0; i < 4; i++)
    {
      Point adj = Point(curr.r + Neighbors[i][row], curr.c + Neighbors[i][col]);

      if (Maze[curr.r][curr.c] & Neighbors[i][dir] && Distance[adj.r][adj.c] == Distance[curr.r][curr.c] - 1)
      {
        curr = adj;
        Path.push(adj);
        break;
      }
    }
  }
  
  return Path;
}

// returns stack of shortest path points from src to dest
PointStack GetShortestPath(const int Maze[ROWS][COLS], const Point src, const Point dest)
{ 
  if (!IsValid(src) || !IsValid(dest))
    return PointStack();
  
  PointQueue Queue;    // store points which are yet to be explored
  Queue.push(src);
  
  int Distance[ROWS][COLS] = {0};   // distance from source for all pts
  bool Visited[ROWS][COLS] = {0};   // visited status of all pts
  Visited[src.r][src.c] = true;

  while (!Queue.isEmpty())
  {
    Point curr = Queue.pop();

    if (curr.r == dest.r && curr.c == dest.c)
      break;

    for (int i = 0; i < 4; i++)
    {
      Point adj = Point(curr.r + Neighbors[i][row], curr.c + Neighbors[i][col]);

      if (Maze[curr.r][curr.c] & Neighbors[i][dir] && !Visited[adj.r][adj.c])  // path to neighbor and unvisited
      {
        Visited[adj.r][adj.c] = true;
        Distance[adj.r][adj.c] = Distance[curr.r][curr.c] + 1;
        Queue.push(adj);
      }
    }
  }

  return BacktracePath(Maze, src, dest, Distance);
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

  switch (GetDirection(src, dest))
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

// travel complete path given stack of neighboring points
void TraversePath(PointStack Stack)
{
  Point prev = Stack.peek();
  
  while (!Stack.isEmpty())
  {
    Point curr = Stack.pop();
    Serial.print(curr.r);
    Serial.print(", ");
    Serial.println(curr.c);
//    MoveToPoint(prev, curr);
    prev = curr;
  }
}

// ---------------------------------------- BLUETOOTH ----------------------------------------

void ReceiveMaze()
{
  while (!Bluetooth.available()) {}   // wait to establish connection

  bool RowReceived = false;
  bool ColReceived = false;

  while (!RowReceived || !ColReceived)
  {
    int val = Bluetooth.read();

    if (val >= 0)
    {
      if (!RowReceived)
      {
        StartPoint.r = val;
        RowReceived = true;
      }

      if (!ColReceived)
      {
        StartPoint.c = val;
        ColReceived = true;
      }
    }
  }

  int i = 0;

  while (true)
  {
    int val = Bluetooth.read();

    if (val >= 0)
    {
      Maze[i/5][i%5] = val;
      i++;
    }

    if (i == 25)  // maze data completely received
      break;
  }
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

  Serial.begin(9600);

//  Bluetooth.begin(9600);
//  ReceiveMaze();

  for (int i = 0; i < 5; i++)
  {
    for (int j = 0; j < 5; j++)
    {
      Maze[i][j] = SampleMaze[i][j];
      Serial.print(SampleMaze[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }

  delay(2500);
  TraversePath(GetShortestPath(Maze, StartPoint, BottomRightCorner));
  delay(2500);
  TraversePath(GetShortestPath(Maze, TopLeftCorner, TopRightCorner));
  delay(2500);
  TraversePath(GetShortestPath(Maze, TopRightCorner, BottomRightCorner));
  delay(2500);
  TraversePath(GetShortestPath(Maze, BottomRightCorner, BottomLeftCorner));
}

void loop() {}
