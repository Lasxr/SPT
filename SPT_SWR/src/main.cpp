#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs =
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS,
                    TCS34725_GAIN_4X);

#define MR_IN1 14       // Back Right Motor
#define MR_IN2 27
#define MR_IN3 26    // Front Right Motor
#define MR_IN4 25       

#define ML_IN1 23    // Front Left Motor
#define ML_IN2 12
#define ML_IN3 33   // Back Left Motor
#define ML_IN4 32

// Motor Driver Pins
#define BR_ENA 16
#define FR_ENB 19
#define BL_ENB 4
#define FL_ENA 15


// Motor Speed
uint8_t BR_Speed = 120; 
uint8_t BL_Speed = 125; 
uint8_t FR_Speed = 120;
uint8_t FL_Speed = 125; 

// IR Sensor Pins
#define IR_FL 18
#define IR_FR 17                                                                                                                                                                                                                                                                                      
#define IR_BL 34                                                                                                                                                                                                                                                                                  
#define IR_BR 35 

bool ir_FL, ir_FR, ir_BL, ir_BR;


#define SERVO_R 5
#define SERVO_L 13

#define SERVO_FREQ 50
#define SERVO_RES 16

uint32_t dutyMin = 3277;  // 0
uint32_t dutyMid = 4915;  // 90
uint32_t dutyMax = 6554;  // 180


bool RED = 0;
bool BLUE = 0;
bool GREEN = 0;
bool YELLOW = 0;



void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Motor Pins
  pinMode(MR_IN1, OUTPUT);  // Back Right Motor
  pinMode(MR_IN2, OUTPUT); 
  pinMode(MR_IN3, OUTPUT);   // Front Right Motor
  pinMode(MR_IN4, OUTPUT);

  pinMode(ML_IN1, OUTPUT);  // Front Left Motor
  pinMode(ML_IN2, OUTPUT);
  pinMode(ML_IN3, OUTPUT);  // Back Left Motor
  pinMode(ML_IN4, OUTPUT);

  pinMode(BR_ENA, OUTPUT);
  pinMode(FR_ENB, OUTPUT);
  pinMode(BL_ENB, OUTPUT);
  pinMode(FL_ENA, OUTPUT);

  // IR Sensor Pins
  pinMode(IR_FL, INPUT);
  pinMode(IR_FR, INPUT);
  pinMode(IR_BL, INPUT);
  pinMode(IR_BR, INPUT);

  analogWrite(BR_ENA, BR_Speed); // Back Right Motor
  analogWrite(FR_ENB, FR_Speed); // Front Right Motor
  analogWrite(BL_ENB, BL_Speed); // Back Left Motor
  analogWrite(FL_ENA, FL_Speed); // Front Left Motor

  ledcSetup(0, SERVO_FREQ, SERVO_RES);
  ledcSetup(1, SERVO_FREQ, SERVO_RES);

  ledcAttachPin(SERVO_R, 0);
  ledcAttachPin(SERVO_L, 1);

  delay(1000);

  if (!tcs.begin()) {
    Serial.println("Not found TCS3472");
    while (1);
  }
  
  Serial.println("Program Start");

}


void Forward(int Delay) {
  Serial.println("Forward");
  digitalWrite(MR_IN1, HIGH);  // Back Right Motor
  digitalWrite(MR_IN2, LOW);
  digitalWrite(MR_IN3, HIGH);   // Front Right Motor
  digitalWrite(MR_IN4, LOW);

  digitalWrite(ML_IN1, HIGH);  // Front Left Motor
  digitalWrite(ML_IN2, LOW);
  digitalWrite(ML_IN3, HIGH);  // Back Left Motor
  digitalWrite(ML_IN4, LOW);

  delay(Delay);
}

void Backward(int Delay) {

  Serial.println("Backward");
  digitalWrite(MR_IN1, LOW);  // Back Right Motor
  digitalWrite(MR_IN2, HIGH);
  digitalWrite(MR_IN3, LOW);   // Front Right Motor
  digitalWrite(MR_IN4, HIGH);

  digitalWrite(ML_IN1, LOW);  // Front Left Motor
  digitalWrite(ML_IN2, HIGH);
  digitalWrite(ML_IN3, LOW);  // Back Left Motor
  digitalWrite(ML_IN4, HIGH);

  delay(Delay);
}

void Left(int Delay) {
  Serial.println("Left");
  digitalWrite(MR_IN1, LOW);  // Back Right Motor
  digitalWrite(MR_IN2, HIGH);
  digitalWrite(MR_IN3, HIGH);   // Front Right Motor
  digitalWrite(MR_IN4, LOW);

  digitalWrite(ML_IN1, LOW);  // Front Left Motor
  digitalWrite(ML_IN2, HIGH);
  digitalWrite(ML_IN3, HIGH);  // Back Left Motor
  digitalWrite(ML_IN4, LOW);

  delay(Delay);
}

void Right(int Delay) {
  Serial.println("Right");
  digitalWrite(MR_IN1, HIGH);  // Back Right Motor
  digitalWrite(MR_IN2, LOW);
  digitalWrite(MR_IN3, LOW);   // Front Right Motor
  digitalWrite(MR_IN4, HIGH);

  digitalWrite(ML_IN1, HIGH);  // Front Left Motor
  digitalWrite(ML_IN2, LOW);
  digitalWrite(ML_IN3, LOW);  // Back Left Motor
  digitalWrite(ML_IN4, HIGH);

  delay(Delay);
}

void StopCar(int Delay) {
  Serial.println("Stop");
  digitalWrite(MR_IN1, LOW);  // Back Right Motor
  digitalWrite(MR_IN2, LOW);
  digitalWrite(MR_IN3, LOW);   // Front Right Motor
  digitalWrite(MR_IN4, LOW);

  digitalWrite(ML_IN1, LOW);  // Front Left Motor
  digitalWrite(ML_IN2, LOW);
  digitalWrite(ML_IN3, LOW);  // Back Left Motor
  digitalWrite(ML_IN4, LOW);

  delay(Delay);
}

void BackLeft(int Delay) {
  Serial.println("Back Left");
  digitalWrite(MR_IN1, LOW);  // Back Right Motor
  digitalWrite(MR_IN2, HIGH);
  digitalWrite(MR_IN3, LOW);   // Front Right Motor
  digitalWrite(MR_IN4, LOW);

  digitalWrite(ML_IN1, LOW);  // Front Left Motor
  digitalWrite(ML_IN2, HIGH);
  digitalWrite(ML_IN3, LOW);  // Back Left Motor
  digitalWrite(ML_IN4, LOW);

  delay(Delay);
}

void BackRight(int Delay) {
  Serial.println("Back Right");
  digitalWrite(MR_IN1, LOW);  // Back Right Motor
  digitalWrite(MR_IN2, LOW);
  digitalWrite(MR_IN3, LOW);   // Front Right Motor
  digitalWrite(MR_IN4, HIGH);

  digitalWrite(ML_IN1, LOW);  // Front Left Motor
  digitalWrite(ML_IN2, LOW);
  digitalWrite(ML_IN3, LOW);  // Back Left Motor
  digitalWrite(ML_IN4, HIGH);

  delay(Delay);
}

void ForwardLeft(int Delay) {
  Serial.println("Forward Left");
  digitalWrite(MR_IN1, LOW);  // Back Right Motor
  digitalWrite(MR_IN2, LOW);
  digitalWrite(MR_IN3, HIGH);   // Front Right Motor
  digitalWrite(MR_IN4, LOW);

  digitalWrite(ML_IN1, LOW);  // Front Left Motor
  digitalWrite(ML_IN2, LOW);
  digitalWrite(ML_IN3, HIGH);  // Back Left Motor
  digitalWrite(ML_IN4, LOW);

  delay(Delay);
}

void ForwardRight(int Delay) {
  Serial.println("Forward Right");
  digitalWrite(MR_IN1, HIGH);  // Back Right Motor
  digitalWrite(MR_IN2, LOW);
  digitalWrite(MR_IN3, LOW);   // Front Right Motor
  digitalWrite(MR_IN4, LOW);

  digitalWrite(ML_IN1, HIGH);  // Front Left Motor
  digitalWrite(ML_IN2, LOW);
  digitalWrite(ML_IN3, LOW);  // Back Left Motor
  digitalWrite(ML_IN4, LOW);

  delay(Delay);
}

void TurnLeft(int Delay) {
  Serial.println("Turn Left");


  digitalWrite(MR_IN1, HIGH);  // Back Right Motor
  digitalWrite(MR_IN2, LOW);
  digitalWrite(MR_IN3, HIGH);   // Front Right Motor
  digitalWrite(MR_IN4, LOW);

  digitalWrite(ML_IN1, LOW);  // Front Left Motor
  digitalWrite(ML_IN2, HIGH);
  digitalWrite(ML_IN3, LOW);  // Back Left Motor
  digitalWrite(ML_IN4, HIGH); 

  delay(Delay);
}

void TurnRight(int Delay) {

  Serial.println("Turn Right");
  digitalWrite(MR_IN1, LOW);  // Back Right Motor
  digitalWrite(MR_IN2, HIGH);
  digitalWrite(MR_IN3, LOW);   // Front Right Motor
  digitalWrite(MR_IN4, HIGH);

  digitalWrite(ML_IN1, HIGH);  // Front Left Motor
  digitalWrite(ML_IN2, LOW);
  digitalWrite(ML_IN3, HIGH);  // Back Left Motor
  digitalWrite(ML_IN4, LOW);

  delay(Delay);
}

void setServoAngle(uint8_t ch, uint8_t angle) {
  uint32_t duty = map(angle, 0, 180, 3277, 6554);
  ledcWrite(ch, duty);
}


void read_ir() {
  while (1) {
    Serial.print("FL : ");
    Serial.print(digitalRead(IR_FL));
    Serial.print("      FR value: ");
    Serial.print(digitalRead(IR_FR));
    //////////////////////////////////////
    Serial.print("      BL : ");
    Serial.print(digitalRead(IR_BL));
    Serial.print("      BR value: ");
    Serial.println(digitalRead(IR_BR));
    delay(100);
  }
}

void val_IR(){
  ir_FL = digitalRead(IR_FL);
  ir_FR = digitalRead(IR_FR);
  ir_BL = digitalRead(IR_BL);
  ir_BR = digitalRead(IR_BR);
}

void read_RGB(){
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (r > g && r > b && r < 500) {
    Serial.println("RED");
  }else if (g > r && g > b && g < 500) {
    Serial.println("GREEN");
  }else if (b > r && b > g && b < 500) {
    Serial.println("BLUE");
  }else if (r > 500 && g > 500 && b < 500) {
  Serial.println("YELLOW");
  }else {
    Serial.println("WHITE");
  }
}

void val_RGB(){
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (r > g && r > b && r < 500) {
    RED = 1;
    BLUE = 0;
    GREEN = 0;
    YELLOW = 0;
  }
  else if (g > r && g > b && g < 500)
  {
    RED = 0;
    BLUE = 0;
    GREEN = 1;
    YELLOW = 0;
  }
  else if (b > r && b > g && b < 500)
  {
    RED = 0;
    BLUE = 1;
    GREEN = 0;
    YELLOW = 0;
  }
  else if (r > 500 && g > 500 && b < 500)
  {
    RED = 0;
    BLUE = 0;
    GREEN = 0;
    YELLOW = 1;
  }
  else
  {
    RED = 0;
    BLUE = 0;
    GREEN = 0;
    YELLOW = 0;
  }
}

void To_FrontLine() {
  while (1)
  {
    val_IR();
    if (ir_FL == 1 && ir_FR == 1)
    {
      StopCar(100);
      break;
    }else if (ir_FL == 0 && ir_FR == 0)
    {
      Forward(0);
    }else if (ir_FL == 1 && ir_FR == 0)
    {
      TurnLeft(0);
    }else if (ir_FL == 0 && ir_FR == 1)
    {
      TurnRight(0);
    }
  }
  Backward(200);
  StopCar(100);
}

void To_LeftLine(){
  while (1)
  {
    val_IR();
    if (ir_FL == 1 && ir_BL == 1)
    {
      StopCar(100);
      break;
    }else if (ir_FL == 0 && ir_BL == 0)
    {
      Left(0);
    }else if (ir_FL == 1 && ir_BL == 0)
    {
      TurnRight(0);
    }else if (ir_FL == 0 && ir_BL == 1)
    {
      TurnLeft(0);
    }
  }
  Right(300);
  StopCar(100);
  
}

void To_RightLine(){
  while (1)
  {
    val_IR();
    if (ir_FR == 1 && ir_BR == 1)
    {
      StopCar(100);
      break;
    }else if (ir_FR == 0 && ir_BR == 0)
    {
      Right(0);
    }else if (ir_FR == 1 && ir_BR == 0)
    {
      TurnLeft(0);
    }else if (ir_FR == 0 && ir_BR == 1)
    {
      TurnRight(0);
    }
  }
  Left(300);
  StopCar(100);
  
}

void Box_Right(){
  Forward(800);
  StopCar(300);
  while (1)
  {
    val_IR();
    if (ir_FR == 1 && ir_BR == 1)
    {
      StopCar(100);
      break;
    }else if (ir_FR == 0 && ir_BR == 0)
    {
      Right(0);
    }else if (ir_FR == 1 && ir_BR == 0)
    {
      TurnRight(0);
    }else if (ir_FR == 0 && ir_BR == 1)
    {
      TurnLeft(0);
    }
  }
  Left(350);
  StopCar(200);
}

void Box_Left(){
  Forward(800);
  StopCar(300);
  
  while (1)
  {
    val_IR();
    if (ir_FL == 1 && ir_BL == 1)
    {
      StopCar(100);
      break;
    }else if (ir_FL == 0 && ir_BL == 0)
    {
      Left(0);
    }else if (ir_FL == 1 && ir_BL == 0)
    {
      TurnRight(0);
    }else if (ir_FL == 0 && ir_BL == 1)
    {
      TurnLeft(0);
    }
  }
  Right(350);
  StopCar(200);
}

void End_proses(){
  while (1)
  {
    StopCar(0);
  }
  
}

void loop(){
  setServoAngle(0, 90);
  setServoAngle(1, 90);
  delay(3000);
  val_RGB();
  Box_Left();
  Box_Left();
  Box_Left();

  To_FrontLine();
  To_LeftLine();
  TurnRight(600);
  Box_Left();
  Box_Left();

  To_FrontLine();
  To_LeftLine();
  TurnRight(600);
  Box_Left();

  To_FrontLine();
  To_LeftLine();
  TurnRight(600);
  Box_Right();
  
  To_FrontLine();
  To_RightLine();
  TurnLeft(600);

  Box_Right();
  Forward(500);

  End_proses();
}

