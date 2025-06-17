/* ****************************************************************************
Copyright 2025 k-off pacovali@student.42berlin.de

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
**************************************************************************** */

#include <math.h>
#include <stdint.h>
#include "Position.hpp"
#include "Shapes.hpp"

// Plotter controlling pins enumeration
enum ControlPins {
  pinEnableRot    = 8,   // rotary motion motor enable pin (x)
  pinStepRot      = 2,   // rotary motion motor step pin (x)
  pinRotDirection = 5,   // rotary motion motor direction pin (x)
  pinEnableLin    = 8,   // linear motion motor enable pin (y)
  pinStepLin      = 3,   // linear motion motor step pin (y)
  pinLinDirection = 6,   // linear motion motor direction pin (y)
  pinLights       = 11,  // lights enable pin
};

// A4988 motor driver EN pin states
enum MotorState {
  Enable=LOW,
  Disable=HIGH
};

// There are two motor motions in plotter: rotary and linear
enum MotorDirection {
  CW = 0,   // clockwise
  IN = 0,   // towards center
  CCW = 1,  // counter-clockwise
  OUT = 1   // from center
};

constexpr const float rotTotalSteps = 16000.0;
constexpr const float linTotalSteps = 5000.0;
constexpr const float scale = 100.0;

uint16_t  knob1 = 0;  //int knob1 = analogRead(A1);
uint16_t  knob2 = 0;


float r = 0;          // length of extended arm
Pos gCurrPos = {0, 0};
Pos gNextPos = {0, 0}; // only needed for serial output

float xLoc = 0;
float yLoc = 0;

int16_t offset = 0;
int16_t moving = 0;

float radAngle = 0;
float rotAngle = 0;

int16_t linearSteps = 0;   // Y
int16_t linearStepsTo = 0;
int16_t rotSteps = 0;     // X
int16_t rotStepsTo = 0;
uint8_t rotMotorState = Enable;
uint8_t linMotorState = Enable;

// Variables to store results
float destinationRadius, destinationAngle;

int pwm = 0;
int pwm2 = 0;

uint8_t linDirection = OUT;
uint8_t rotDirection = CCW;

int linSpeed = 1000;
int rotSpeed = 500;

void plotShape(const Shape& shape) {
  for (uint16_t i = 0; i < shape.size; i++) {
    gNextPos = shape.data[i];
    gotoPos(shape.data[i]);     //shift data left and down 50 units to make grid 100x100
  }
}

void gotoPos(const Pos& pos) {
  // Calculate difference between current and next positions
  Pos deltaPos = pos - gCurrPos;

  // Determine the total number of steps to move
  int totalSteps = max(abs(deltaPos.x), abs(deltaPos.y)) || 1; // avoid zero-division

  // Determine size of a single increment
  Pos step = deltaPos/totalSteps;

  for (int i = 0; i < totalSteps; i++) {
    gCurrPos += step;
    updateDirection(gCurrPos);
    moveTo(gCurrPos);
  }
}

void display() {
  if (moving == 1) {
    r = (linearSteps / linTotalSteps) * scale;  // changing
    gCurrPos.x = r * cos(radAngle);
    gCurrPos.y = r * sin(radAngle);
  }
  Serial.print(rotDirection);
  Serial.print(",");
  Serial.print(linDirection);
  Serial.print(",");
  Serial.print(rotSpeed);
  Serial.print(",");
  Serial.print(knob2);
  Serial.print(" ");
  Serial.print(offset);
  Serial.print(" *  ");
  Serial.print(linMotorState);
  Serial.print(",");
  Serial.print(rotMotorState);
  Serial.print(" %");
  Serial.print(gCurrPos.x);
  Serial.print("x,y");
  Serial.print(gCurrPos.y);
  Serial.print(" ");
  Serial.print(rotSteps);
  Serial.print(">");
  Serial.print(rotStepsTo);
  Serial.print(" ");
  Serial.print(linearSteps);
  Serial.print(">");
  Serial.print(linearStepsTo);
  Serial.print(" (");
  Serial.print(gNextPos.x);
  Serial.print(",");
  Serial.print(gNextPos.y);
  Serial.println("");
}


void updateDirection(const Pos& pos) {
  destinationRadius = pos.radialDistance();
  destinationAngle = atan2(pos.y, pos.x);
  if (destinationAngle < 0) {
    destinationAngle = (2 * PI) + destinationAngle;
  }

  rotStepsTo = (destinationAngle / (2 * PI)) * rotTotalSteps;
  linearStepsTo = (destinationRadius / scale) * linTotalSteps;

  if (linearSteps > linearStepsTo) { linDirection = IN; }
  if (linearSteps < linearStepsTo) { linDirection = OUT; }
  digitalWrite(pinLinDirection, linDirection);


  if ((rotStepsTo > rotSteps && (rotStepsTo - rotSteps < rotTotalSteps / 2)) || (rotStepsTo < rotSteps && (rotSteps - rotStepsTo > rotTotalSteps / 2))) {
    rotDirection = CCW;
    digitalWrite(pinRotDirection, rotDirection);
  } else {
    rotDirection = CW;
    digitalWrite(pinRotDirection, rotDirection);
  }
}


// Function to move to a specific target position
void moveTo(const Pos& pos) {
  gNextPos = pos;
  updateDirection(pos);
  if (rotSteps != rotStepsTo) rotMotorState = 1;
  if (linearSteps != linearStepsTo) linMotorState = 1;
  while (rotMotorState == 1 || linMotorState == 1) {
    // DO NOTHING!!! wait until hits rot Target location.
    delay(1);
  }
}


void setup() {
  //Setup Interupts
  TCCR1A = 0;  // Init Timer1A
  TCCR1B = 0;  // Init Timer1B
  TCCR1B |= B00000011;  // Prescaler = 64
  OCR1A = 1000;         // Timer Compare1A Register  // = 16000000 / (64 * 1000) - 1 (must be <65536)
  OCR1B = 1000;         // Timer Compare1B Register
  TIMSK1 |= B00000110;  // only use one interupt

  // Define pins as outputs
  pinMode(pinStepRot, OUTPUT);
  pinMode(pinRotDirection, OUTPUT);
  pinMode(pinStepLin, OUTPUT);
  pinMode(pinLinDirection, OUTPUT);
  pinMode(pinEnableLin, OUTPUT);
  pinMode(pinEnableRot, OUTPUT);
  pinMode(pinLights, OUTPUT);

  digitalWrite(pinLights, HIGH);

  digitalWrite(pinEnableLin, Enable);
  digitalWrite(pinEnableRot, Enable);

  Serial.begin(9600);
  Serial.println("sTARting ");
  display();
}

void loop() {
  if (rotMotorState == Enable || linMotorState == Enable) {
    display();
    digitalWrite(pinLights, Enable);
  }
  else {
    digitalWrite(pinLights, Disable);
  }

  knob1 = analogRead(A0);
  knob2 = analogRead(A1);
  rotSpeed = map(knob1, 0, 1023, 100, 2000);
  linSpeed = map(knob2, 0, 1023, 150, 1000);
  if (knob2>1015) {
    linMotorState=Enable;
    if (rotMotorState==0) moving=0;
    digitalWrite(pinEnableLin, HIGH);
  }
  else {
    linMotorState=1;
    moving=1;
  }


if (knob1>1015) 
{
rotMotorState=0;
 if (linMotorState==0) moving=0;
 digitalWrite(pinEnableRot, HIGH);
}
else
{
rotMotorState=1;
moving=1;
}


  if (Serial.available() > 0) {
    // Read the incoming byte
    char command = Serial.read();

    // Move X axis

    if (command == 'q') {  // turn on x motor
      rotMotorState = !rotMotorState;
      if (linMotorState==0) moving = !moving;
      digitalWrite(pinEnableRot, !rotMotorState);  // Enable the stepper motor
    }

    if (command == 'w') {               // change dir of x motor
      digitalWrite(pinRotDirection, rotDirection);  // Set direction
      rotDirection = !rotDirection;
    }


    if (command == 'e') {  // change dir of x motor

      rotSpeed += 50;
      //digitalWrite(pinRotDirection,rotDirection); // Set direction
      // rotDirection=!rotDirection;
    }
    if (command == 'r') {  // change dir of x motor

      rotSpeed -= 50;
      //digitalWrite(pinRotDirection,rotDirection); // Set direction
      // rotDirection=!rotDirection;
    }


    






    if (command == 'a') {  // turn on x motor
      linMotorState = !linMotorState;
      if (rotMotorState==0) moving = !moving;
      digitalWrite(pinEnableLin, !linMotorState);  // Enable the stepper motor
    }

    if (command == 's') {                 // change dir of x motor
      digitalWrite(pinLinDirection, linDirection);  // Set direction
      linDirection = !linDirection;
    }
    if (command == 'd') {  // change dir of linear motion motor
      linSpeed -= 50;
    }
    if (command == 'f') {  // change dir of linear motion motor
      linSpeed += 50;
    }
    if (command == 'x') {
      plotShape(gStar);
    }
    if (command == 'z') {
      plotShape(gLine);
    }


    // Rotate axis
    if (command == 'Y') {
      digitalWrite(pinLinDirection, HIGH);  // Set direction
      for (int i = 0; i < 200; i++) {    // Steps for 360 degree rotation
        digitalWrite(pinStepLin, HIGH);
        delayMicroseconds(500);  // Adjust delay for speed
        digitalWrite(pinStepLin, LOW);
        delayMicroseconds(500);  // Adjust delay for speed
      }
    }
  }
}

// First interupt
ISR(TIMER1_COMPA_vect) {
  OCR1A += rotSpeed;  // Advance The COMPA Register

  

if (rotMotorState==0&&linMotorState==0)offset=0;

  if (rotMotorState == 1) {


//only if one is rotating 
    digitalWrite(pinEnableRot, LOW);  // Enable the stepper motor
  pwm = !pwm;

    digitalWrite(pinStepRot, pwm);

  if (pwm == 0&&rotMotorState==1&&linMotorState==1)
    offset += 1;
  if (offset > 9 ) {
    if (rotDirection == 1) linearSteps =linearSteps+ 1;// only increase 1/10 the speed
    if (rotDirection == 0) linearSteps =linearSteps- 1;// only increase 1/10 the speed
    offset=0;
  }

      if (rotDirection == 1) rotSteps = rotSteps + 1;
      if (rotDirection == 0)rotSteps = rotSteps - 1;

     

      if (rotSteps > 16000) rotSteps = 0;  //happens only once

      if (rotSteps < 0) rotSteps = 15999;                  //happens only once
      radAngle = (rotSteps / rotTotalSteps) * (2 * PI);  /// changing
      rotAngle = (rotSteps / rotTotalSteps) * 360.0;



      
      if (rotSteps == rotStepsTo && moving == 0)  rotMotorState = 0;  // turn off motor if it reaches the set point
    }
}

// Linear motion motor: timer interrupt service routine - 
ISR(TIMER1_COMPB_vect) {  // speed for second motor

  OCR1B += linSpeed;  // Advance The COMPB Register //8750

  if (linMotorState == 1) {

    digitalWrite(pinEnableLin, LOW);  // Enable the stepper motor

    pwm2 = !pwm2;
    digitalWrite(pinStepLin, pwm2);

    if (pwm2 == 0) {

      if (moving == 0) {
        int state = linDirection;
        if (linearSteps > linearStepsTo) { linDirection = 0; }  // go in
        if (linearSteps < linearStepsTo) { linDirection = 1; }  // go out
                                                       // if (state!=linDirection) linearSteps+=135;// a change has occured.
        digitalWrite(pinLinDirection, !linDirection);            // Set direction
      }
      if (linDirection == 1) linearSteps = linearSteps + 1;
      if (linDirection == 0) linearSteps = linearSteps - 1;

      if (linearSteps > 5000 && moving == 1) {
        linDirection = 0;
        digitalWrite(pinLinDirection, 1);  // Set direction
      }
      if (linearSteps < 0 && moving == 1) {

        linDirection = 1;
        digitalWrite(pinLinDirection, 0);  // Set direction
      }

      if (linearSteps == linearStepsTo && moving == 0) {
        linMotorState = 0;
        }
    }
  }
}

