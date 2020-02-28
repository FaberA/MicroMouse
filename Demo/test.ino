#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>

Encoder M1Enc(3, 2); //right motor
Encoder M2Enc(6, 7); //left motor

long M1_newPosition = -999;
long M2_newPosition = -999;

const int M1_Logic0 =  0;
const int M1_Logic1 =  1;
const int M2_Logic0 =  5;
const int M2_Logic1 =  4;

 const int ticks_per_rotation = 1000;
  const float wheel_diameter = 2.4;
  const float wheel_circ = 3.14159*wheel_diameter;


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600*8);
}

void forward(int speed, float distance)
{
  M1Enc.write(0);
  M2Enc.write(0);
 
  float rotations_needed = distance/wheel_circ;
  const int tick_goal = ceil(ticks_per_rotation*rotations_needed);

  float timer = millis(); //acts as previous time
  float dt = 0;
  int lm = speed;
  int rm = speed;
  
  int kp = 100; //PID -> kp, ki, kd
  int kd = 600;
  int ki = 600;
  
  int p_err = 0;
  int d_err = 0;
  int i_err = 0;
  signed int prev_err = 0;

  //Serial.println(tick_goal);

  while(M1_newPosition < tick_goal)
  {
    analogWrite(M1_Logic0, 0);
    analogWrite(M1_Logic1, rm);
    analogWrite(M2_Logic0, 0);
    analogWrite(M2_Logic1, lm);
    M1_newPosition = M1Enc.read();
    M2_newPosition = M2Enc.read();

    //dt= millis()-timer;
    //timer = millis();
    
    p_err = M2_newPosition - M1_newPosition; //master - slave
    
    //d_err = (p_err-prev_err) / dt;
    //i_err = (i_err+p_err)*dt; 
    //prev_err = p_err;
    //rm = p_err/kp +d_err*kd + i_err*ki + rm; //edit slave
    rm = p_err/kp + rm; 
    Serial.print(M1_newPosition);
    Serial.print(" ");
    Serial.println(M2_newPosition);

    delay(30);
  }
     analogWrite(M1_Logic0, 0);
    analogWrite(M1_Logic1, 0);
    analogWrite(M2_Logic0, 0);
    analogWrite(M2_Logic1, 0);

    M1Enc.write(0);
    M2Enc.write(0);
    M1_newPosition = 0;
    M2_newPosition = 0;   
}


void reverse(int speed, float distance)
{
  M1Enc.write(0);
  M2Enc.write(0);
 
  float rotations_needed = distance/wheel_circ;
  const int tick_goal = ceil(ticks_per_rotation*rotations_needed);
  
  int lm = speed;
  int rm = speed;
  int kp = 50; //PID -> kp, ki, kd
  int error = 0;
  

  //Serial.println(tick_goal);

  while(M1_newPosition < tick_goal)
  {
    analogWrite(M1_Logic0, rm);
    analogWrite(M1_Logic1, 0);
    analogWrite(M2_Logic0, lm);
    analogWrite(M2_Logic1, 0);
    M1_newPosition = M1Enc.read();
    M2_newPosition = M2Enc.read();

    error = M1_newPosition - M2_newPosition; //master - slave
    rm = error/kp + rm; //edit slave
    
    Serial.print(M1_newPosition);
    Serial.print(" ");
    Serial.println(M2_newPosition);

    delay(20);
  }
     analogWrite(M1_Logic0, 0);
    analogWrite(M1_Logic1, 0);
    analogWrite(M2_Logic0, 0);
    analogWrite(M2_Logic1, 0);

    M1Enc.write(0);
    M2Enc.write(0);
    M1_newPosition = 0;
    M2_newPosition = 0;   
}

void loop() {

  forward(120, 50);

}
