#include <math.h>
#include <Encoder.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <stdlib.h> 
#include <stdbool.h> 
#define ENCODER_USE_INTERRUPTS

#define ENCODER_TICKS 1000
#define WHEEL_DIAM 2.32
#define DRIVE_MAX_PWR 200
#define TURN_PWR 200

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
#define maximum(x,y) (x > y ? x : y > x ? y : x )

Encoder RM_Enc(3, 2); //right motor
Encoder LM_Enc(6, 7); //left motor

LSM9DS1 imu; //gyro object
#define DECLINATION 11 // Declination (degrees) 11 deg umbc

#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

long LM_oldPosition  = 0;
long RM_oldPosition  = 0;
long LM_newPosition = 0;
long RM_newPosition = 0;

const int RM_Logic0 =  0;
const int RM_Logic1 =  1;
const int LM_Logic0 =  5;
const int LM_Logic1 =  4;

const int L0_T = 9;
const int L45_T = 10;
const int L90_T = 11;
const int R90_T = 12;
const int R45_T = 14;
const int R0_T = 15;

const int L0_R = A2; 
const int L45_R = A3;
const int L90_R = A6; 
const int R90_R = A7;
const int R45_R = A8; 
const int R0_R = A9;

//const int MS = 10;
const int DEBUG = 13;

int L45_prev = 0;
int L45_curr = 0;
int R45_prev = 0;
int R45_curr = 0;

long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 100;    // the debounce time; increase if the output flickers
volatile bool Debug_state = true;

// maze solving variables
typedef struct Node {
    int x; 
    int y; 
    int weight;
    bool visited;  
    bool W_D; 
    bool W_U; 
    bool W_L; 
    bool W_R; 
    struct Node* up; 
    struct Node* down; 
    struct Node* left; 
    struct Node* right; 
} Node;
Node** maze;

enum NOSE { RIGHT, LEFT, UP, DOWN };
NOSE nose=UP;

const int X_DIM = 3;
const int Y_DIM = 3;
const int goal_x = 1;
const int goal_y = 0;
int mm_x = 0;
int mm_y = 0;

void reset_visits(){
  for(int i = 0; i < X_DIM; ++i){
    for(int j = 0; j < Y_DIM; ++j){
      maze[i][j].visited=false;
    }
  }
}

void flood(int x, int y, int dist){
  if(x > X_DIM-1 || x < 0 || y > Y_DIM-1 || y < 0) return;
  if(!maze[x][y].visited || (maze[x][y].weight != 0 && dist<maze[x][y].weight)){
    maze[x][y].visited = true;
    maze[x][y].weight = dist;
  }
  else return;
  
  if(!maze[x][y].W_U) flood(x,y+1,dist+1);
  if(!maze[x][y].W_D) flood(x,y-1,dist+1);
  if(!maze[x][y].W_R) flood(x+1,y,dist+1);
  if(!maze[x][y].W_L) flood(x-1,y,dist+1);
}

void setup() {
  Serial.begin(9600);
  
  pinMode(L0_T, OUTPUT);
  pinMode(L45_T, OUTPUT);
  pinMode(L90_T, OUTPUT);
  pinMode(R0_T, OUTPUT);
  pinMode(R45_T, OUTPUT);
  pinMode(R90_T, OUTPUT);
    
  pinMode(RM_Logic0, OUTPUT);
  pinMode(RM_Logic1, OUTPUT);
  pinMode(LM_Logic0, OUTPUT);
  pinMode(LM_Logic1, OUTPUT);

  //pinMode(MS,INPUT);
  pinMode(DEBUG, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DEBUG),debug_flip,CHANGE); 
  
  analogWrite(RM_Logic0, 0);
  analogWrite(RM_Logic0, 0);
  analogWrite(LM_Logic1, 0);
  analogWrite(LM_Logic1, 0);
  
  digitalWrite(L0_T, LOW);
  digitalWrite(R0_T, LOW);
  digitalWrite(L90_T, LOW);
  digitalWrite(R90_T, LOW);
  digitalWrite(L45_T, LOW);
  digitalWrite(R45_T, LOW);

  //Gyro
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1)
      ;
  }

  // initialize maze
  maze = (Node**)malloc(X_DIM*sizeof(Node*));
  for(int i =0; i < Y_DIM; ++i){
    maze[i] = (Node*)malloc(Y_DIM*sizeof(Node));
  }
  
  for(int i = 0; i < X_DIM; ++i){ /// set coordinates & connect nodes
    for(int j =0; j < Y_DIM; ++j){
      maze[i][j].x = i;
      maze[i][j].y = j;
      maze[i][j].weight = 0;
      if(i < X_DIM-1) {
        maze[i][j].right = &maze[i+1][j];
        maze[i][j].W_R = false;
      }
      else {
        maze[i][j].right = NULL;
        maze[i][j].W_R = true;
      }
      if(i > 0){
         maze[i][j].left = &maze[i-1][j];
         maze[i][j].W_L = false;
       }
      else {
        maze[i][j].left = NULL;
        maze[i][j].W_L = true;
      }
      if(j < Y_DIM-1) {
        maze[i][j].up = &maze[i][j+1];
        maze[i][j].W_U = false;
      }
      else {
        maze[i][j].up = NULL;
        maze[i][j].W_U = true;
      }
      if(j > 0){
         maze[i][j].down = &maze[i][j-1];
         maze[i][j].W_D = false;
       }
      else{
         maze[i][j].down = NULL;
         maze[i][j].W_D = true;
       }
    }
  }
}

void debug_flip()
{
  if(digitalRead(DEBUG) == HIGH) Debug_state = true;  
  else Debug_state = false;
}

void turnMotorsoff()
{
  analogWrite(RM_Logic0, 0);
  analogWrite(LM_Logic0, 0);
  analogWrite(RM_Logic1, 0);
  analogWrite(LM_Logic1, 0);
}
  
void forward(int lmspeed, int rmspeed)
{
  analogWrite(RM_Logic0, 0);
  analogWrite(LM_Logic0, 0);
  analogWrite(RM_Logic1, rmspeed);
  analogWrite(LM_Logic1, lmspeed);
}

void reverse(int lmspeed, int rmspeed)
{
  analogWrite(RM_Logic0, rmspeed);
  analogWrite(LM_Logic0, lmspeed);
  analogWrite(RM_Logic1, 0);
  analogWrite(LM_Logic1, 0);
}

void turnL(int lmspeed, int rmspeed)
{
  analogWrite(RM_Logic0, 0);
  analogWrite(LM_Logic0, lmspeed);
  analogWrite(RM_Logic1, rmspeed);
  analogWrite(LM_Logic1, 0);
}

void turnR(int lmspeed, int rmspeed)
{
  analogWrite(RM_Logic0, rmspeed);
  analogWrite(LM_Logic0, 0);
  analogWrite(RM_Logic1, 0);
  analogWrite(LM_Logic1, lmspeed);
}

float L0_distance(void) //get distance from L0 Sensor
{
  float L0_d; 
  int L0_prev = 0;
  int L0_curr = 0;
  
  digitalWrite(L0_T, LOW);
  delay(2);
  L0_prev = analogRead(L0_R);
  digitalWrite(L0_T, HIGH);
  delay(2);
  L0_curr = analogRead(L0_R);
  L0_curr = L0_curr-L0_prev;
  //Serial.print("(adc)");
  //Serial.print(L0_curr);
  //Serial.print("   ");
  L0_d = L0_dist(L0_curr);
  digitalWrite(L0_T, LOW);
  delay(1);
  return L0_d;
}

float R0_distance(void) ////get distance from R0 Sensor
{
  float R0_d;
  int R0_prev = 0;
  int R0_curr = 0;
  
  digitalWrite(R0_T, LOW);
  delay(2);
  R0_prev = analogRead(R0_R);
  digitalWrite(R0_T, HIGH);
  delay(2);
  R0_curr = analogRead(R0_R);
  R0_curr = R0_curr-R0_prev;
  R0_d = R0_dist(R0_curr);
  digitalWrite(R0_T, LOW);
  delay(1);
  return R0_d;
}

float L90_distance(void) //get distance from L90 Sensor
{
  float L90_d; 
  int L90_prev = 0;
  int L90_curr = 0;
  
  digitalWrite(L90_T, LOW);
  delay(2);
  L90_prev = analogRead(L90_R);
  digitalWrite(L90_T, HIGH);
  delay(2);
  L90_curr = analogRead(L90_R);
  L90_curr = L90_curr-L90_prev;
  //  Serial.print("(adc)");
  //Serial.print(L90_curr);
  //Serial.print("   ");
  L90_d = L90_dist((float)L90_curr);
  digitalWrite(L90_T, LOW);
  delay(1);
  return L90_d;
}

float R90_distance(void) //get distance from R90 Sensor
{
  float R90_d; 
  int R90_prev = 0;
  int R90_curr = 0;
  
  digitalWrite(R90_T, LOW);
  delay(2);
  R90_prev = analogRead(R90_R);
  digitalWrite(R90_T, HIGH);
  delay(2);
  R90_curr = analogRead(R90_R);
  R90_curr = R90_curr-R90_prev;
  //Serial.print("(adc)");
  //Serial.print(R90_curr);
  //Serial.print("   ");
  R90_d = R90_dist((float)R90_curr);
  digitalWrite(R90_T, LOW);
  delay(1);
  return R90_d;
}

float front_distance(void) //get average distance from front sensors (L0 & R0)
{
   float L0_d, R0_d;
   L0_d = L0_distance();
   R0_d = R0_distance();
   return ((R0_d+L0_d)/2);
}

void turn_Left(int speed, float deg)
{
  turnMotorsoff();
  float timer = millis();
  float degree = 0;

  while (degree < (deg-3)  && !(Debug_state)) 
  {
    float delta_time = (millis() - timer) / 1000;
    timer = millis();
    imu.readGyro();
    degree += imu.calcGyro(imu.gz) * delta_time;

    turnL(speed, speed);
    delay(2);
  }
  turnMotorsoff();
}

void turn_Right(int speed, float deg)
{
  turnMotorsoff();
  float timer = millis();
  float degree = 0;

  while (degree > (-deg+13)  && !(Debug_state)) 
  {
    float delta_time = (millis() - timer) / 1000;
    timer = millis();
    imu.readGyro();
    degree += imu.calcGyro(imu.gz) * delta_time;

    turnR(speed, speed);
    delay(2);
  }
  turnMotorsoff();
}

void driveDistance(float dist) //drives straight for a distance
{
  //reset encoders
  RM_Enc.write(0);
  LM_Enc.write(0);
  int drive_LastError = 0;

  int drive_target = ceil(fabs((dist/(WHEEL_DIAM*3.14159))*ENCODER_TICKS)); //tick goal
  int drive_error = drive_target - LM_Enc.read();

  int DRIVE_ERROR_THRESH = 500;
  int Kp = 6;
  int Kd = 4;
  
  while (abs(drive_error) > DRIVE_ERROR_THRESH) // while more than DRIVE_ERROR_THRESH from target
  {
    drive_error = drive_target - abs(LM_Enc.read()); // update error value
    int drivePower = sgn(dist)*(Kp * drive_error) + (Kd * (drive_error - drive_LastError)); // initial PD power calculation

    drivePower = abs(drivePower) > DRIVE_MAX_PWR ? sgn(drivePower) * DRIVE_MAX_PWR : drivePower; // limit power if calc is over max

    // drive straight power modifier
    int rDiff = abs(LM_Enc.read()) - abs(RM_Enc.read()); // check for right encoder difference
    int rMod = sgn(rDiff)*drivePower*.1;          // rMod = 10% drivePower in the direction of rDiff

    forward( drivePower , drivePower + rMod );

    drive_LastError = drive_error; // update driveLastError
    delay(2);
  } 
  turnMotorsoff();
}

void driveDistance_parallel(float dist) //uses side walls to keep driving straight and uses encoders
{
  //reset encoders
  RM_Enc.write(0);
  LM_Enc.write(0);
  int drive_LastError = 0;

  int drive_target = ceil(fabs((dist/(WHEEL_DIAM*3.14159))*ENCODER_TICKS)); //tick goal
  int drive_error = drive_target - LM_Enc.read();

  float WALL_THRESH = .9; //.9 cm difference between side sensors used as error threshold
  int DRIVE_ERROR_THRESH = 500; //correction to scale encoder count
  int Kp = 6;
  int Kd = 4;

  float stop_dist, left_dist, right_dist, parallel_error;
  
  while ((abs(drive_error) > DRIVE_ERROR_THRESH)) // while more than DRIVE_ERROR_THRESH from target
  {
    stop_dist = front_distance();
    if(stop_dist < 3 || Debug_state)
    {
      turnMotorsoff();
      delay(10);
      break;  
    }
    
    drive_error = drive_target - abs(LM_Enc.read()); // update error value
    int drivePower = sgn(dist)*(Kp * drive_error) + (Kd * (drive_error - drive_LastError)); // initial PD power calculation

    drivePower = abs(drivePower) > DRIVE_MAX_PWR ? sgn(drivePower) * DRIVE_MAX_PWR : drivePower; // limit power if calc is over max

    left_dist = L90_distance();
    right_dist = R90_distance();
    if(left_dist < 8 && right_dist < 8) {
      parallel_error = left_dist-right_dist;
      WALL_THRESH = .3;
    }
    else if(left_dist < 8 && right_dist >= 8){
       parallel_error = left_dist - 1.5;
       WALL_THRESH = .3;
    }
    else if(left_dist >= 8 && right_dist < 8){
      parallel_error = 2 - right_dist;
      WALL_THRESH = .3;
    }
    else {
      //parallel_error = abs(abs(LM_Enc.read()) - abs(RM_Enc.read()));
      parallel_error = 2;
      WALL_THRESH = 99;
    }
    // drive straight power modifier
    int rDiff;
    int rMod;
    if(fabs(parallel_error) > WALL_THRESH)
    {
      int save_enc = LM_Enc.read();   
      rDiff = abs(LM_Enc.read()) - (abs(RM_Enc.read()-500*(int)parallel_error)); //Alter encoder difference depending on side wall proximity
      rMod = sgn(rDiff)*drivePower*.3;
      forward( drivePower , drivePower + rMod );
      LM_Enc.write(0);
      RM_Enc.write(0);
      delay(25);
      int avg = (LM_Enc.read()+RM_Enc.read())/2; //average encoder ticks for front distance from angled correction
      LM_Enc.write(save_enc+avg);
      RM_Enc.write(save_enc+avg);
    }
    else { // no walls on left or right side
    // drive straight power modifier
    rDiff = abs(LM_Enc.read()) - abs(RM_Enc.read()); // check for right encoder difference
    rMod = sgn(rDiff)*drivePower*.1;          // rMod = 10% drivePower in the direction of rDiff
    forward( drivePower , drivePower + rMod );
    drive_LastError = drive_error; // update driveLastError
    delay(2);
    }
  } 
  turnMotorsoff();
  delay(10);
}

Node* next_node(){
  Node* mm_node = &maze[mm_x][mm_y];
  Node* dest = NULL;
  int temp_w; // used to check weights of legit moves
  if(mm_y < Y_DIM-1 && mm_node->W_U==false){
    temp_w = mm_node->up->weight; // start with upper node's weight
    dest = mm_node->up;
  }
  else if(mm_x < X_DIM-1 && mm_node->W_R==false){
    temp_w = mm_node->right->weight;
    dest = mm_node->right;
  }
  else if(mm_y > 0 && mm_node->W_D==false){
    temp_w = mm_node->down->weight;
    dest = mm_node->down;
  }
  else{
    temp_w = mm_node->left->weight;
    dest = mm_node->left;
  }
  if(mm_node->right != NULL){
    if(temp_w > mm_node->right->weight && mm_node->W_R == false){ // choose least weight
      temp_w=mm_node->right->weight;
      dest=mm_node->right;
    }
  }
  if(mm_node->down != NULL){
    if(temp_w > mm_node->down->weight && mm_node->W_D == false){
      temp_w=mm_node->down->weight;
      dest=mm_node->down;
    }
  }
  if(mm_node->left != NULL){
    if(temp_w > mm_node->left->weight && mm_node->W_L == false){
      temp_w=mm_node->left->weight;
      dest=mm_node->left;
    }
  }
  if(mm_node->up != NULL){
    if(temp_w > mm_node->up->weight && mm_node->W_U == false){
      temp_w=mm_node->up->weight;
      dest=mm_node->up;
    }
  }
  return dest;
}

void print_maze(){
  for(int q = 0; q < X_DIM; ++q){ // top walls
    if(q==0 && maze[q][Y_DIM-1].W_U==true) Serial.print("    -----   ");
    else Serial.print("  -----   ");
  }
  Serial.print("\n");
  int down_walls[3] = {0,0,0}; // x_dim
  for(int j = Y_DIM-1; j >= 0; --j){
    for(int i = 0; i < X_DIM; ++i){
      if(i == 0 && maze[i][j].W_L==true) Serial.print(" | ");
      else if(i==0) Serial.print("   ");
      Serial.print("(");
      Serial.print(maze[i][j].x);
      Serial.print(",");
      Serial.print(maze[i][j].y);
      Serial.print(",");
      Serial.print(maze[i][j].weight);
      Serial.print(")");
      if(maze[i][j].W_R==true) Serial.print(" | ");
      else Serial.print("   ");
      if(maze[i][j].W_D==true) down_walls[i] = 1;
      if(i == X_DIM-1){
        Serial.print("\n");
        for(int q = 0; q < X_DIM; ++q){
          if(down_walls[q]==1){ 
            if(q==0) Serial.print("    -----   ");
            else Serial.print("  -----   ");
          }
          else Serial.print("          ");
        }
        for(int q = 0; q < X_DIM; ++q) down_walls[q]=0;
        Serial.print("\n");
      }
    }
  }
  Serial.println(nose);
  Serial.print(mm_x);
  Serial.println(mm_y);
  //delay(2000);
}

void update_Walls(){
  Serial.println("In update");
  switch(nose){
    case UP:
      Serial.println("In up");
      if(L90_distance() < 8){
        maze[mm_x][mm_y].W_L = true; 
        if(mm_x >0) maze[mm_x-1][mm_y].W_R = true;
      }
      if(R90_distance() < 8){
        maze[mm_x][mm_y].W_R = true; 
        if(mm_x < X_DIM-1) maze[mm_x+1][mm_y].W_L = true;
      }
      if(front_distance() < 8){
        maze[mm_x][mm_y].W_U = true; 
        if(mm_y < Y_DIM-1) maze[mm_x][mm_y+1].W_D = true;
      }
   break;
   case DOWN:
      if(L90_distance() < 8){
        maze[mm_x][mm_y].W_R = true; 
        if(mm_x < X_DIM-1) maze[mm_x+1][mm_y].W_L = true;
      }
      if(R90_distance() < 8){
        maze[mm_x][mm_y].W_L = true; 
        if(mm_x > 0) maze[mm_x-1][mm_y].W_R = true;
      }
      if(front_distance() < 8){
        maze[mm_x][mm_y].W_D = true; 
        if(mm_y > 0) maze[mm_x][mm_y-1].W_U = true;
      }
   break;
   case LEFT:
      if(L90_distance() < 8){
        maze[mm_x][mm_y].W_D = true; 
        if(mm_y > 0) maze[mm_x][mm_y-1].W_U = true;
      }
      if(R90_distance() < 8){
        maze[mm_x][mm_y].W_U = true; 
        if(mm_y < Y_DIM-1) maze[mm_x][mm_y+1].W_D = true;
      }
      if(front_distance() < 8){
        maze[mm_x][mm_y].W_L = true; 
        if(mm_x > 0) maze[mm_x-1][mm_y].W_R = true;
      }
   case RIGHT:
      if(L90_distance() < 8){
        maze[mm_x][mm_y].W_U = true; 
        if(mm_y < Y_DIM-1) maze[mm_x][mm_y+1].W_D = true;
      }
      if(R90_distance() < 8){
        maze[mm_x][mm_y].W_D = true; 
        if(mm_y > 0) maze[mm_x][mm_y-1].W_U = true;
      }
      if(front_distance() < 8){
        maze[mm_x][mm_y].W_R = true; 
        if(mm_x < X_DIM-1) maze[mm_x+1][mm_y].W_L = true;
      }
  }
}

void loop() {
   if(!Debug_state){ // solve maze
      if(mm_x==goal_x && mm_y==goal_y) for(;;) //done
      Serial.println("before update");
      update_Walls();
      Serial.println("\nPRINTING UPDATE");
      //print_maze();
      reset_visits();
      flood(goal_x,goal_y,0);
      print_maze();
      Node* dest = next_node();
      int dest_x = dest->x;
      int dest_y = dest->y;
      //print_maze();
      //Serial.print(dest_x);
      //Serial.println(dest_y);
      //delay(10000);
      switch(nose){
        case UP:
          if(dest_x-mm_x==0){
            if(dest_y-mm_y==1)mm_y++; // go up
            else{ // go down
              turn_Left(TURN_PWR, 180); //power, deg
              mm_y--;
              nose = DOWN;
            }
          }
          else if(dest_y-mm_y==0){
            if(dest_x-mm_x==1){ // go right
              turn_Right(TURN_PWR,90);
              mm_x++;
              nose = RIGHT;
            }
            else{ // go left
              turn_Left(TURN_PWR, 90); //power, deg
              mm_x--;
              nose = LEFT;
            }
          }
        break;
        case DOWN:
          if(dest_x-mm_x==0){
            if(dest_y-mm_y==1){
              turn_Left(TURN_PWR,180);
              mm_y++; // go up
              nose = UP;
            }
            else mm_y--;
          }
          else if(dest_y-mm_y==0){
            if(dest_x-mm_x==1){ // go right
              turn_Left(TURN_PWR,90);
              mm_x++;
              nose = RIGHT;
            }
            else{ // go left
              turn_Right(TURN_PWR, 90); //power, deg
              mm_x--;
              nose = LEFT;
            }
          }
        break;
        case LEFT:
          if(dest_x-mm_x==0){
            if(dest_y-mm_y==1){
              turn_Right(TURN_PWR,90);
              mm_y++; // go up
              nose = UP;
            }
            else{ // go down
              turn_Left(TURN_PWR, 90); //power, deg
              mm_y--;
              nose = DOWN;
            }
          }
          else if(dest_y-mm_y==0){
            if(dest_x-mm_x==1){ // go right
              turn_Right(TURN_PWR,180);
              mm_x++;
              nose = RIGHT;
            }
            else{
              mm_x--;
            }
          }
        break;
        case RIGHT:
          if(dest_x-mm_x==0){
            if(dest_y-mm_y==1){
              turn_Left(TURN_PWR,90);
              mm_y++; // go up
              nose = UP;
            }
            else{
              turn_Right(TURN_PWR, 90); //power, deg
              mm_y--;
              nose = DOWN;
            }
          }
          else if(dest_y-mm_y==0){
            if(dest_x-mm_x==1) mm_x++;
            else{ // go left
              turn_Left(TURN_PWR, 180); //power, deg
              mm_x--;
              nose = LEFT;
            }
          }
        break;
      }
      if(front_distance() < 20) driveDistance_parallel(36);
      else driveDistance_parallel(20);
      //delay(1000); 
      delay(2);
   }
   else {
    turnMotorsoff();
    print_maze();
    delay(2000);
    Serial.println("\n");
   }
}
