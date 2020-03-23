#include <math.h>
#include <Encoder.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#define ENCODER_USE_INTERRUPTS

#define ENCODER_TICKS 1000
#define WHEEL_DIAM 2.32
#define DRIVE_MAX_PWR 210

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

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

  while (degree < (deg-7.18)  && !(Debug_state)) 
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
    if(stop_dist < 18 || Debug_state)
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
    parallel_error = left_dist-right_dist;
    
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
    else 
    {
    rDiff = abs(LM_Enc.read()) - abs(RM_Enc.read()); // check for right encoder difference  
    rMod = sgn(rDiff)*drivePower*.1;          // rMod = 10% drivePower in the direction of rDiff
    forward( drivePower , drivePower + rMod );
    drive_LastError = drive_error; // update driveLastError
    delay(2);
    }
  } 
  turnMotorsoff();
}

void loop() {
   if(!Debug_state){
     driveDistance_parallel(120);
     turn_Left(180, 90);
     delay(1000);
   }
   else {
    turnMotorsoff();
   }
}
