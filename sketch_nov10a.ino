// Add these in the arduino ide
#include <Servo.h> 
#include <Wire.h>
#include "Adafruit_TCS34725.h"
// S1 IS LOW AND S0 IS HIGH FOR BOTH
#define S2LEFT 4
#define S3LEFT 7
#define sensorOutLEFT A1

#define WOODTHRESHOLDLEFT 20
#define FOLLOWSPEED 160
#define REACHSPEED 150
#define TURNSPEED 160

#define WOODTHRESHOLDRIGHT 0.2
 
// Variables for Color Pulse Width Measurements
 
int redPW = 0;
int greenPW = 0;
int bluePW = 0;
 
#define IRSensor A7



// motor boi stuff
#define LEFTMOTORSPEED 3
#define RIGHTMOTORSPEED 6
#define LEFTMOTORIN2 9
#define RIGHTMOTORIN2 5
#define LEFTMOTORIN 11
#define RIGHTMOTORIN 8

#define GRABBER 10

// CONSTANTS
#define IRThreshold 250
#define GRABDIST 450
#define IRMAX 590
#define DELAYTICK 10
#define DEGREES_PER_MS 0.1715
#define CLOSE_VALUE 87
#define OPEN_VALUE 5

Servo Servo1; 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
bool isDone = false;

void leftMotorForward(int speed){
  digitalWrite(LEFTMOTORIN, HIGH);
  digitalWrite(LEFTMOTORIN2, LOW);
  analogWrite(LEFTMOTORSPEED, speed-20);
}

void leftMotorBackward(int speed){
  digitalWrite(LEFTMOTORIN2, HIGH);
  digitalWrite(LEFTMOTORIN, LOW);
  analogWrite(LEFTMOTORSPEED, speed-20);
}

void rightMotorForward(int speed){
  digitalWrite(RIGHTMOTORIN, HIGH);
  digitalWrite(RIGHTMOTORIN2, LOW);
  analogWrite(RIGHTMOTORSPEED, speed);
}

void rightMotorBackward(int speed){
  digitalWrite(RIGHTMOTORIN2, HIGH);
  digitalWrite(RIGHTMOTORIN, LOW);
  analogWrite(RIGHTMOTORSPEED, speed);
}

void forward(int speed){
  rightMotorBackward(speed);
  leftMotorBackward(speed);
}

void backward(int speed){
  rightMotorForward(speed);
  leftMotorForward(speed);
}

void turnRight(int rightSpeed, int leftSpeed){
  rightMotorForward(rightSpeed);
  leftMotorBackward(leftSpeed);
}

void turnLeft(int rightSpeed, int leftSpeed){
  rightMotorBackward(rightSpeed);
  leftMotorForward(leftSpeed);
}

void leftMotorOff(){
  digitalWrite(LEFTMOTORIN2, LOW);
  digitalWrite(LEFTMOTORIN, LOW);
  analogWrite(LEFTMOTORSPEED, 0);
}

void rightMotorOff(){
  digitalWrite(RIGHTMOTORIN2, LOW);
  digitalWrite(RIGHTMOTORIN, LOW);
  analogWrite(RIGHTMOTORSPEED, 0);
}
 
// Function to read Red Pulse Widths
int getRedPW(int S2, int S3,int sensorOut) {
 
  // Set sensor to read Red only
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
 
}
 
// Function to read Green Pulse Widths
int getGreenPW(int S2, int S3,int sensorOut) {
 
  // Set sensor to read Green only
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
 
}
 
// Function to read Blue Pulse Widths
int getBluePW(int S2, int S3,int sensorOut) {
 
  // Set sensor to read Blue only
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
 
}

void stop(){
    leftMotorOff();
    rightMotorOff();
}

bool isWoodBoi(int red, int green, int blue, float redProp,float greenProp, float blueProp, String side){
  if(side == "right"){
      if(abs(red - green) > WOODTHRESHOLDLEFT){
        return false;
      }
      if(abs(red - blue) > WOODTHRESHOLDLEFT){
        return false;
      }
      if(abs(green - blue) > WOODTHRESHOLDLEFT){
        return false;
      }
      return true;
  }
  else{
    if(abs(redProp - greenProp) > WOODTHRESHOLDRIGHT){
        return false;
      }
      if(abs(redProp - blueProp) > WOODTHRESHOLDRIGHT){
        return false;
      }
      if(abs(greenProp - blueProp) > WOODTHRESHOLDRIGHT){
        return false;
      }
      return true;
  }

  
}
// red is 1, blue is 2 green is 3
int getColor(String sensor, uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c){
    // assume circle boi is on left if not swap
    if(sensor == "right"){
        redPW = getRedPW(S2LEFT,S3LEFT, sensorOutLEFT);
        delay(4);
        bluePW = getBluePW(S2LEFT,S3LEFT, sensorOutLEFT);
        delay(4);
        greenPW = getGreenPW(S2LEFT,S3LEFT, sensorOutLEFT);
        delay(4);  
        if(isWoodBoi(redPW,greenPW,bluePW, 0,0,0,"right")){
          return -1;
        }
        if(redPW <= bluePW && redPW <= greenPW){
          return 1;
        }
        else if(bluePW <= redPW && bluePW <= greenPW){
          return 2; 
        }
        else if(greenPW <= redPW && greenPW <= bluePW){
          return 3;
        }
        else{
          return -2;
        } 
    }
    else{
          tcs.getRawData(&r, &g, &b, &c);
          float total = r + b + g;
          float rf = 0;
          float gf = 0;
          float bf = 0;
          rf = r/total;
          gf = g/total;
          bf = b/total;
          if(isWoodBoi(0,0,0,rf,gf,bf, "left")){
            return -1;
          }
          if(rf >= bf && rf >= gf){
            return 1;
          }
          else if(bf >= rf && bf >= gf){
            return 2; 
          }
          else if(gf >= rf && gf >= bf){
            return 3;
          }
          else{
            return -2;
          } 
    }


}

void reachMan(){
    // go forward until max
    int IR = IRThreshold;
    while(IR < IRMAX){
      // correct for if it gets off track
      if(IR < IRThreshold){
        findTarget();
        backward(REACHSPEED);
        delay(50);
      }
      else{
        backward(REACHSPEED);
        delay(50);
      }
      IR = analogRead(IRSensor);
    }
    // # tight polling for the distance
    while(analogRead(IRSensor) > GRABDIST){}
    stop();
}

bool turn(String direction, float degree, bool findMan){
    stop();
    if(direction == "left"){
        turnLeft(TURNSPEED,TURNSPEED);
    }
    else{
        turnRight(TURNSPEED,TURNSPEED);
    }
    float rotation = 0;
    while(rotation < degree){
        if(findMan && analogRead(IRSensor) > IRThreshold){
            stop();
            return true;
        }
        delay(DELAYTICK);
        rotation += DEGREES_PER_MS*DELAYTICK;
    }
    stop();
    return false;
}

void wiggle(String prev,uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c){
  int col1 = 1;
  int col2 = 1;
  stop();
  while(col1 == 1 && col2 == 1){
    col1 = getColor("right",r,g,b,c);
    col2 = getColor("left",r,g,b,c);
    if(prev == "left"){
      turnLeft(TURNSPEED,20);
    }
    else{
      turnRight(20,TURNSPEED);
    }
  }
  stop();
}
void followPath(int color, uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c){
    int col1 = -1;
    int col2 = -1;
    String prev = "";
    if(color == 2 || color == 3){
        while(col1 != color && col2 != color ){
            col1 = getColor("right",r,g,b,c);
            col2 = getColor("left",r,g,b,c);
            if(col1 != 1 && col2 != 1){
                forward(FOLLOWSPEED);
            }
            else if(col1 == 1 && col2 != 1){
                turnRight(TURNSPEED-20,TURNSPEED);
                prev = "right";
            }
            else if(col2 == 1 && col1 != 1){
                turnLeft(TURNSPEED,20);
                prev = "left";
            }
            else{
              wiggle(prev,r,g,b,c);
            }
        }
    }
    // for detecting the one in the middle
    else if(color == 4){
      while(true){
            col1 = getColor("right",r,g,b,c);
            col2 = getColor("left",r,g,b,c);
            if(col1 == 3 && col2 == 3){
              break;
            }
            if(col1 != 1 && col2 != 1){
                forward(FOLLOWSPEED);
            }
            else if(col1 == 1 && col2 != 1){
                turnRight(20,TURNSPEED);
                prev = "right";
            }
            else if(col2 == 1 && col1 != 1){
                turnLeft(TURNSPEED,20);
                prev = "left";
            }
            else{
             wiggle(prev,r,g,b,c);
            }
        }
    }
    else{
      const int maxRed = 5;
      int redCount = 0;
      while(redCount <  maxRed){
            col1 = getColor("right",r,g,b,c);
            col2 = getColor("left",r,g,b,c);
            if(col1 != 1 && col2 != 1){
                forward(FOLLOWSPEED);
                redCount = 0;
            }
            else if(col1 == 1 && col2 != 1){
                turnRight(20,TURNSPEED);
                prev = "right";
                redCount = 0;
            }
            else if(col2 == 1 && col1 != 1){
                turnLeft(TURNSPEED,TURNSPEED-20);
                prev = "left";
                redCount = 0;
            }
            else{
              if(prev == "left"){
                turnRight(20,TURNSPEED);
                delay(50);
              }
              else{
                turnLeft(TURNSPEED,TURNSPEED-20);
                delay(50);
              }
              redCount++;
            }
        }
    }

    stop();
}
void findTarget(){
    // the area we want to scan
    int SCAN_CONE = 45;
    // # start at 45 degrees to the left and scan 90 degrees to the left
    bool found = turn("left", SCAN_CONE, true);
    if(found){
    }
    else{
        turn("right",SCAN_CONE*2,true);
    }
}
void grabTarget(){
    Servo1.write(CLOSE_VALUE);
}


void alignWithPath(float forwardTime, uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c){
    forward(REACHSPEED);
    delay(forwardTime*1000);
    stop();

//     turn left and find red line
    float rotation = 0;
    turnLeft(REACHSPEED,REACHSPEED);
    while(rotation < 45){
        delay(DELAYTICK);
        rotation += DEGREES_PER_MS*DELAYTICK;
        if(getColor("right",r,g,b,c)== 1){
            stop();
            return;
        }
    }

    // not found in left turn must turn 90 degrees in other way
    rotation = 0;
    turnRight(REACHSPEED,REACHSPEED);

    while(rotation < 90){
        delay(DELAYTICK);
        rotation += DELAYTICK*DEGREES_PER_MS;
        if(getColor("left",r,g,b,c) == 1){
            stop();
            return;
        }
    }
}

void dropTarget(){
    Servo1.write(OPEN_VALUE);
}




void setup() {
  Serial.begin(9600);

    pinMode(S2LEFT, OUTPUT);
    pinMode(S3LEFT, OUTPUT);
    
    // Set Sensor output as input
    pinMode(sensorOutLEFT, INPUT);

    
    pinMode(IRSensor, INPUT);


    pinMode(LEFTMOTORIN2, OUTPUT);
    pinMode(RIGHTMOTORIN2, OUTPUT);
    pinMode(LEFTMOTORIN, OUTPUT);
    pinMode(RIGHTMOTORIN, OUTPUT);
    pinMode(LEFTMOTORSPEED, OUTPUT);
    pinMode(RIGHTMOTORSPEED, OUTPUT);
    pinMode(GRABBER, OUTPUT);

    Servo1.attach(GRABBER); 
    dropTarget();
}

void testSensor(String side, uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c){
  Serial.print(side);
  Serial.print(":\n");
  Serial.print(getColor(side,r,g,b,c));
  Serial.print("\n");
  delay(500);
}

void alignWithMiddle(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c){
  turnLeft(TURNSPEED,TURNSPEED);
  while(getColor("right",r,g,b,c)!= 1){}
  unsigned long tim = millis();
  turnRight(TURNSPEED,TURNSPEED);
  while(getColor("left",r,g,b,c)!= 1){}
  tim = millis()-tim;
  tim /= 2;
  tim += 0;
  turnLeft(TURNSPEED,TURNSPEED);
  delay(tim);
  stop();
}

void loop() {
  uint16_t r, g, b, c;
    if(!isDone){
        followPath(2,r,g,b,c);
        // must reposition
        backward(FOLLOWSPEED);
        delay(150);
        stop();
        turn("left",180,false);
        reachMan();
        delay(200);
        grabTarget();
        alignWithPath(0,r,g,b,c);
        followPath(1,r,g,b,c);
        turn("left",180,false);
        stop();
        dropTarget();
        isDone = true;
    }
    else{
    }
}
