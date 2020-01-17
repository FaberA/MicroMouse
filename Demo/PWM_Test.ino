#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

Encoder M1Enc(5, 4);
Encoder M2Enc(6, 7);

LSM9DS1 imu; //gyro object
#define DECLINATION 11 // Declination (degrees) 11 deg umbc

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

const int M1_Logic0 =  0;
const int M1_Logic1 =  1;
const int M2_Logic0 =  2;
const int M2_Logic1 =  3;

const int analogFrontPin = A0;
const int analogRightPin = A1;
const int analogLeftPin = A2;

int frontADC = 0;
int leftADC = 0;
int rightADC = 0;

long M1_oldPosition  = -999;
long M2_oldPosition  = -999;
long M1_newPosition = 0;
long M2_newPosition = 0;

void setup()   {
  //motor output
  pinMode(M1_Logic0, OUTPUT);
  pinMode(M1_Logic1, OUTPUT);
  pinMode(M2_Logic0, OUTPUT);
  pinMode(M2_Logic1, OUTPUT);

  //gyro
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

  Serial.begin(9600);
  //Serial.println("Lm,gyro,Rm");
}

void forward(int speed)
{
  frontADC = analogRead(analogFrontPin);

  float gyroerror = 0;
  int lm = speed;
  int rm = speed;
  int kp = 40;
  int error = 0;
  float timer = millis(); //acts as previous time
  float deltatime = 0;

  for (int x = 0; x < 50; ++x)
  {
    frontADC = analogRead(analogFrontPin);
    if (frontADC >= 500)
    {
      analogWrite(M1_Logic0, 0);
      analogWrite(M2_Logic0, 0);
      analogWrite(M1_Logic1, 0);
      analogWrite(M2_Logic1, 0);
      return;
    }
    analogWrite(M1_Logic0, 0);
    analogWrite(M2_Logic0, 0);
    analogWrite(M1_Logic1, lm);
    analogWrite(M2_Logic1, rm);

    M1_newPosition = M1Enc.readAndReset();
    M2_newPosition = M2Enc.readAndReset();

    error = M1_newPosition - M2_newPosition;
    //Serial.print(M1_newPosition);
    //Serial.print(" ");
    //Serial.println(M2_newPosition);
    rm += error / kp;

    if ( imu.gyroAvailable() )
    {
      deltatime = (millis()-timer)/1000;
      timer = millis();
      imu.readGyro();
      gyroerror += imu.calcGyro(imu.gz)*deltatime;
    }

    if (gyroerror > 1)
    {
      lm += 1;
      rm -= 1;
    }
    else if (gyroerror < -1)
    {
      rm += 1;
      lm -= 1;
    }
    delay(10);
  }
}

void reverse(int speed)
{
  analogWrite(M1_Logic0, speed);
  analogWrite(M2_Logic0, speed);
  analogWrite(M1_Logic1, 0);
  analogWrite(M2_Logic1, 0);
}

void turn180(int speed)
{
  float timer = millis();
  float degree = 0;
  //dest_deg = 180;
  //kp = 40;

  while (degree < 180) //165 was pretty good
  {
    float delta_time = (millis() - timer) / 1000;
    timer = millis();
    imu.readGyro();
    degree += imu.calcGyro(imu.gz) * delta_time;

    analogWrite(M1_Logic0, speed);
    analogWrite(M2_Logic0, 0);
    analogWrite(M1_Logic1, 0);
    analogWrite(M2_Logic1, speed);
    //Serial.println(delta_time);
    //error = dest_deg-degree;
    delay(10);
  }
  analogWrite(M1_Logic0, 0);
  analogWrite(M2_Logic0, 0);
  analogWrite(M1_Logic1, 0);
  analogWrite(M2_Logic1, 0);
}


void loop()
{
  //frontADC = analogRead(analogFrontPin);

  //Serial.println(frontADC);


  forward(160)
  if (frontADC < 500)
  {
    forward(160);
  }
  else
  {
    //turn around
    analogWrite(M1_Logic0, 0);
    analogWrite(M2_Logic0, 0);
    analogWrite(M1_Logic1, 0);
    analogWrite(M2_Logic1, 0);
    turn180(240);
    analogWrite(M1_Logic0, 0);
    analogWrite(M2_Logic0, 0);
    analogWrite(M1_Logic1, 0);
    analogWrite(M2_Logic1, 0);
    delay(1000);
  }
  
  /*
   //COOL MOTOR TEST PROGRAMS
    for(int x = 80; x < 256; ++x)
    {
    forward(x);
    delay(20);
    M1_newPosition = M1Enc.read();
    M2_newPosition = M2Enc.read();
    Serial.print(M1_newPosition);
    Serial.print(" ");
    Serial.println(M2_newPosition);
  */
  /*if (M1_newPosition != M1_oldPosition) {
    M1_oldPosition = M1_newPosition;
    Serial.println(M1_newPosition);
    }
    if (M2_newPosition != M2_oldPosition) {
    M2_oldPosition = M2_newPosition;
    Serial.println(M2_newPosition);
    }*/

  /*
    }
    for(int x = 255; x >= 80; --x)
    {
    forward(x);
    delay(20);
    M1_newPosition = M1Enc.read();
    M2_newPosition = M2Enc.read();
    Serial.print(M1_newPosition);
    Serial.print(" ");
    Serial.println(M2_newPosition);
    }
    for(int x = 80; x < 256; ++x)
    {
    reverse(x);
    delay(20);
    M1_newPosition = M1Enc.read();
    M2_newPosition = M2Enc.read();
    Serial.print(M1_newPosition);
    Serial.print(" ");
    Serial.println(M2_newPosition);
    }
    for(int x = 255; x >= 80; --x)
    {
    reverse(x);
    delay(20);
    M1_newPosition = M1Enc.read();
    M2_newPosition = M2Enc.read();
    Serial.print(M1_newPosition);
    Serial.print(" ");
    Serial.println(M2_newPosition);
    }

  */
}
