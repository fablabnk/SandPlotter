#include <math.h>
// Pins


// circle size radius = 100 units

#define enPin_rot 12      // Stepper motor enable pin
#define lights_pin 11      // Stepper motor enable pin
#define stepPin_rot 2    // X axis step pin
#define dirPin_rot 5     // X axis direction pin
#define enPin_InOut 8    // Stepper motor enable pin
#define stepPin_InOut 3  // Y axis step pin
#define dirPin_InOut 6   // Y axis direction pin
#define PI 3.1415926535897932384626433832795


// Changable variables.
#define rot_total_steps 16000.0
#define inOut_total_steps 5000.0
#define desired_scale 100.0

//int knob1 = analogRead(A1);
int knob1 = 0;
int knob2 = 0;


volatile float r = 0;  // length of extended zarm
volatile float x = 0;
volatile float y = 0;  // current location of ball

volatile float x2 = 0;
volatile float y2 = 0;

//int relStep=0;

float xLoc = 0;
float yLoc = 0;

int offset = 0;
int moving = 0;

//volatile float xTarget = 0;
//volatile float yTarget = 0;  // current location of ball



float radAngle = 0;
float rotAngle = 0;


int inOutSteps = 0;  /// Y
int inOutStepsTo = 0;
int rotSteps = 0;  //  X
int rotStepsTo = 0;
byte rotON = 0;
byte inOutON = 1;



// Variables to store results
float rTo, angleTo;
//float targetX = 0;  // Default target coordinates
//float targetY = 0;



//int xAng = 0;

int pwm = 0;
int pwm2 = 0;

byte inOut = 1;  // 1=OUT, 0=IN
byte ccwCW = 1;  // 1 CCW 0=CW

int speed_InOut = 1000;  // speed of second motor
int speed_rot = 500;   // speed of first motor

// change in distanceq

int dx, dy;  //delta x and y
int go = 0;
float m = 0;  //slope between points
float m2 = 0;
float mx, my;  //move distance



int line[] = {
  50, 0,
  0, -50,
  -50, 0,
  0, 50
};

int star[]={
 0,0,
-71,-71,
-71,-71,
-69,-73,
-99,5,
-100,5,
-100,10,
-99,15,
-98,20,
-97,25,
-96,29,
-95,31,
-50,-88,
-50,-88,
-45,-90,
-41,-92,
-36,-94,
-33,-95,
-88,48,
-88,48,
-85,52,
-83,56,
-80,60,
-80,61,
-18,-99,
-18,-99,
-13,-100,
-8,-100,
-3,-100,
-70,71,
-70,71,
-67,75,
-63,78,
-60,80,
11,-100,
11,-100,
16,-99,
20,-98,
24,-98,
-48,87,
-48,87,
-44,90,
-39,92,
-37,93,
36,-94,
36,-94,
41,-92,
45,-90,
48,-88,
-24,97,
-24,97,
-19,98,
-14,99,
-11,99,
59,-80,
59,-81,
63,-78,
67,-75,
69,-72,
3,99,
3,99,
8,99,
12,99,
17,98,
79,-61,
79,-61,
82,-57,
85,-53,
87,-49,
33,93,
33,94,
38,92,
42,90,
46,88,
50,86,
94,-32,
94,-32,
96,-27,
97,-22,
98,-17,
99,-13,
99,-8,
69,71


};



void dir() {
  // Determine how to go IN or OUT?
  if (inOutSteps > inOutStepsTo) { inOut = 0; }  // go in
  if (inOutSteps < inOutStepsTo) { inOut = 1; }  // go out
  digitalWrite(dirPin_InOut, !inOut);            // Set direction


  if ((rotStepsTo > rotSteps && (rotStepsTo - rotSteps < rot_total_steps / 2)) || (rotStepsTo < rotSteps && (rotSteps - rotStepsTo > rot_total_steps / 2))) {
    ccwCW = 1;
    digitalWrite(dirPin_rot, !ccwCW);  // Set direction
  } else {
    ccwCW = 0;
    digitalWrite(dirPin_rot, !ccwCW);  // Set direction
  }
}


void plotShape(int shape[], int size) {

  for (int t = 0; t < (size / 2); t++)  // size of array div 2.
  {
    // if (button > 0) { return; }
    x2 = shape[t * 2];
    y2 = shape[t * 2 + 1];
    Serial.print(x2);
    Serial.print(",");
    Serial.println(y2);
    gotoXY(x2, y2);//shift data left and down 50 units to make gride x100 by y100
    
  }
}

void gotoXY(float x3, float y3) {
  // Calculate the step increments in x and y directions
  float dx = x3 - x;
  float dy = y3 - y;

  // Determine the total number of steps to move
  int totalSteps = max(abs(dx), abs(dy));

  // Calculate the step increments for x and y
  float stepX = dx / totalSteps;
  float stepY = dy / totalSteps;

  // Calculate initial position
  float currentX = x;
  float currentY = y;

  // Move incrementally to the new position
  for (int i = 0; i < totalSteps; i++) {
    // Calculate the new target steps for inout motor
    float newX = currentX + stepX;
    float newY = currentY + stepY;
    reverseKinematics(newX, newY);

    // Move to the next position
    moveTo(newX, newY);

    // Adjust current position for the next iteration
    currentX = newX;
    currentY = newY;
   
  }
   x=x3;
    y=y3;
}





void display() {

  if (moving == 1) {
    r = (inOutSteps / inOut_total_steps) * desired_scale;  // changing
    x = r * cos(radAngle);
    y = r * sin(radAngle);
  }
  //Serial.print("m");
  //Serial.print(m);

 Serial.print(ccwCW);
  Serial.print(",");
  Serial.print(inOut);
  Serial.print(",");
  Serial.print(speed_rot);
  Serial.print(",");
  Serial.print(knob2);
  Serial.print(" ");
  
  //Serial.print("m");
  //Serial.print(m);


 Serial.print(offset);
  Serial.print(" *  ");
  Serial.print(inOutON);
  Serial.print(",");
  Serial.print(rotON);
  Serial.print(" *  ");
  Serial.print(" m");
  Serial.print(m);
  Serial.print(" m2");
  Serial.print(m2);
  //Serial.print(" go");
  //Serial.print(go);

  Serial.print(" %");


  Serial.print(x);
  Serial.print("x,y");
  Serial.print(y);
  Serial.print(" ");


  // Serial.print("rotAng");
  //Serial.print("  ");
  //Serial.print((rotAngle));
  //Serial.print(" ");
  //Serial.print((angleTo));
  //Serial.print(" ");
  //Serial.print("rad=");
  //Serial.print((radAngle));
  //Serial.print("  ");
  //Serial.print("ySpeed");
  //Serial.print("  ");
  // Serial.print(speed_InOut);
  //  Serial.print("  ");




  Serial.print(rotSteps);
  Serial.print(">");

  Serial.print(rotStepsTo);
  Serial.print(" ");
  Serial.print(inOutSteps);
  Serial.print(">");

  Serial.print(inOutStepsTo);
  Serial.print(" (");
  Serial.print(x2);
  Serial.print(",");
  Serial.print(y2);
  Serial.println("");
}



// Function to calculate reverse kinematics
void reverseKinematics(float xx, float yy) {
  // Calculate radial distance (r)
  rTo = sqrt(xx * xx + yy * yy);
  //rTo=rTo* (rot_total_steps/desired_scale);
  // Calculate angle (theta) using atan2
  // atan2 returns the angle in radians in the range -pi to pi
  angleTo = atan2(yy, xx);
  if (angleTo < 0) angleTo = (2 * PI) + angleTo;  // if neg

  //rotAngle=(ccwCW_Steps/16000.0)*360.0;
  rotStepsTo = (angleTo / (2 * PI)) * rot_total_steps;
  inOutStepsTo = (rTo / desired_scale) * inOut_total_steps;


  //(turn on the direction )
  //if (inOutSteps != inOutStepsTo) inOutON = 1;
  //if (rotSteps!= rotStepsTo) rotON = 1;



  // Determine how to go IN or OUT?
  if (inOutSteps > inOutStepsTo) { inOut = 0; }  // go in
  if (inOutSteps < inOutStepsTo) { inOut = 1; }  // go out
  digitalWrite(dirPin_InOut, !inOut);            // Set direction


  if ((rotStepsTo > rotSteps && (rotStepsTo - rotSteps < rot_total_steps / 2)) || (rotStepsTo < rotSteps && (rotSteps - rotStepsTo > rot_total_steps / 2))) {
    ccwCW = 1;
    digitalWrite(dirPin_rot, !ccwCW);  // Set direction
  } else {
    ccwCW = 0;
    digitalWrite(dirPin_rot, !ccwCW);  // Set direction
  }
}


// Function to move to a specific target position
void moveTo(float x4, float y4) {


  x2 = x4;
  y2 = y4;
  reverseKinematics(x4, y4);



  //first rotate then extend
  //while (rotSteps!= rotStepsTo)
  //{
  //rotON = 1;
  //display();
  // }

  if (rotSteps != rotStepsTo) rotON = 1;
   if (inOutSteps != inOutStepsTo) inOutON = 1;
while (rotON == 1||inOutON == 1) {
    // DO NOTHING!!! wait until hits rot Target location.
   // display();
    delay(1);
  }
  


  //delay(10);

  //while (inOutSteps != inOutStepsTo)
  //if (inOutSteps != inOutStepsTo) inOutON = 1;
  //while (inOutON == 1) {
  // DO NOTHING!!! wait until hits inOut Target location.
  //  display();
  // delay(1);
  //}
  // delay(10);

  ////if (rotSteps!= rotStepsTo) rotON = 1;
  //x=xTarget;
  //y=yTarget;
}


void setup() {


  //Setup Interupts
  TCCR1A = 0;  // Init Timer1A
  TCCR1B = 0;  // Init Timer1B
  //TCNT1 = 0;            // initialize counter value to 0
  TCCR1B |= B00000011;  // Prescaler = 64
  OCR1A = 1000;         // Timer Compare1A Register  // = 16000000 / (64 * 1000) - 1 (must be <65536)
  OCR1B = 1000;         // Timer Compare1B Register
  //TIMSK1 |= B00000110;  // Enable Timer COMPA(y)   +COMPB (x) Interrupts
  TIMSK1 |= B00000110;  // only use one interupt

  // Define pins as outputs
  pinMode(stepPin_rot, OUTPUT);
  pinMode(dirPin_rot, OUTPUT);
  pinMode(stepPin_InOut, OUTPUT);
  pinMode(dirPin_InOut, OUTPUT);
  pinMode(enPin_InOut, OUTPUT);
  pinMode(enPin_rot, OUTPUT);

  pinMode(lights_pin, OUTPUT);

digitalWrite(lights_pin, HIGH);  // DIS-Enable the stepper motor

  digitalWrite(enPin_InOut, HIGH);  // DIS-Enable the stepper motor
  digitalWrite(enPin_rot, HIGH);    // DIS-Enable the stepper motor

  // digitalWrite(enPin_InOut, LOW);  // Enable the stepper motor
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("sTARting ");
  display();
}

void loop() {


  //display if motors are moving

  if (rotON == 1 || inOutON == 1) {display(); digitalWrite(lights_pin, HIGH);}  // DIS-Enable the stepper motor
  else digitalWrite(lights_pin, LOW); 

  knob1 = analogRead(A0);
  knob2 = analogRead(A1);

  speed_rot = 2000; // map(knob1, 0, 1023, 100, 2000);
  speed_InOut = 1000; // map(knob2, 0, 1023, 150, 1000);
if (knob2>1015) 
{
inOutON=0;
 if (rotON==0) moving=0;
 digitalWrite(enPin_InOut, HIGH);
}
else
{
inOutON=1;
moving=1;
//digitalWrite(enPin_InOut, LOW);
}


if (knob1>1015) 
{
rotON=0;
 if (inOutON==0) moving=0;
 digitalWrite(enPin_rot, HIGH);
}
else
{
rotON=1;
moving=1;
//digitalWrite(enPin_rot, LOW);
}






  //if (inOutSteps != inOutStepsTo) {

  //dir();
  // Determine how to go IN or OUT?
  //if (inOutSteps > inOutStepsTo) { inOut = 0; }  // go in
  //if (inOutSteps < inOutStepsTo) { inOut = 1; }  // go out
  //digitalWrite(dirPin_InOut, inOut);            // Set direction

  //     inOutON = 1;}


  if (Serial.available() > 0) {
    // Read the incoming byte
    char command = Serial.read();

    // Move X axis




    if (command == 'q') {  // turn on x motor
      rotON = !rotON;
      if (inOutON==0) moving = !moving;
      digitalWrite(enPin_rot, !rotON);  // Enable the stepper motor
    }

    if (command == 'w') {               // change dir of x motor
      digitalWrite(dirPin_rot, ccwCW);  // Set direction
      ccwCW = !ccwCW;
    }


    if (command == 'e') {  // change dir of x motor

      speed_rot += 50;
      //digitalWrite(dirPin_rot,ccwCW); // Set direction
      // ccwCW=!ccwCW;
    }
    if (command == 'r') {  // change dir of x motor

      speed_rot -= 50;
      //digitalWrite(dirPin_rot,ccwCW); // Set direction
      // ccwCW=!ccwCW;
    }


    






    if (command == 'a') {  // turn on x motor
      inOutON = !inOutON;
      if (rotON==0) moving = !moving;
      digitalWrite(enPin_InOut, !inOutON);  // Enable the stepper motor
    }

    if (command == 's') {                 // change dir of x motor
      digitalWrite(dirPin_InOut, inOut);  // Set direction
      inOut = !inOut;
    }


    if (command == 'd') {  // change dir of x motor

      speed_InOut -= 50;
      //digitalWrite(dirPin_rot,ccwCW); // Set direction
      // ccwCW=!ccwCW;
    }
    if (command == 'f') {  // change dir of x motor

      speed_InOut += 50;
      //digitalWrite(dirPin_rot,ccwCW); // Set direction
      // ccwCW=!ccwCW;
    }

    if (command =='x')
    {
      plotShape(star, sizeof(star) / sizeof(star[0]));
    }

    if (command == 'z') {

plotShape(line, sizeof(line) / sizeof(line[0]));
      //moveTo(40, 0);
    //  gotoXY(57, 0);
     // x = 57;
      //y = 0;
      
     // gotoXY(0, 57);
      
    //  gotoXY(-57, 0);
    //  gotoXY(0, -57);
     // gotoXY(57, 0);
      
      
     // gotoXY(0, 0);
      
    }


    // Move Y axis
    if (command == 'Y') {
      //digitalWrite(enPin, LOW); // Enable the stepper motor
      digitalWrite(dirPin_InOut, HIGH);  // Set direction
      for (int i = 0; i < 200; i++) {    // Steps for 360 degree rotation
        digitalWrite(stepPin_InOut, HIGH);
        delayMicroseconds(500);  // Adjust delay for speed
        digitalWrite(stepPin_InOut, LOW);
        delayMicroseconds(500);  // Adjust delay for speed
      }
      //  digitalWrite(enPin, HIGH); // Disable the stepper motor
    }
  }
}






// First interupt
ISR(TIMER1_COMPA_vect) {
  OCR1A += speed_rot;  // Advance The COMPA Register

  

if (rotON==0&&inOutON==0)offset=0;

  if (rotON == 1) {


//only if one is rotating 
    digitalWrite(enPin_rot, LOW);  // Enable the stepper motor
  pwm = !pwm;

    digitalWrite(stepPin_rot, pwm);
   


  

 if (pwm == 0&&rotON==1&&inOutON==1) 
 
 offset += 1;
if (offset > 9 )  

{
  if (ccwCW == 1) inOutSteps =inOutSteps+ 1;// only increase 1/10 the speed
  if (ccwCW == 0) inOutSteps =inOutSteps- 1;// only increase 1/10 the speed
  offset=0;
  
}


    /**
        if (offset==0) digitalWrite(stepPin_InOut, pwm);
    


    if (pwm == 0) {

      offset += 1;
if (offset > 9 )  // only increase 1/10 the speed

{
offset = 0;



 if (ccwCW==1) 
 {
   if (inOutON==0)digitalWrite(dirPin_InOut, ccwCW);  // Set one step
   if (inOutON==1&&rotON==1)inOutSteps+=1;
 }
  if (ccwCW==0) 
  {
    
    //inOutSteps-=1;
    if (inOutON==0)digitalWrite(dirPin_InOut, ccwCW);  // Set one step
    if (inOutON==1&&rotON==1)inOutSteps-=1;
  }

}

*/

      if (ccwCW == 1) rotSteps = rotSteps + 1;
      if (ccwCW == 0)rotSteps = rotSteps - 1;

     

      if (rotSteps > 16000) rotSteps = 0;  //happens only once

      if (rotSteps < 0) rotSteps = 15999;                  //happens only once
      radAngle = (rotSteps / rot_total_steps) * (2 * PI);  /// changing
      rotAngle = (rotSteps / rot_total_steps) * 360.0;



      
      if (rotSteps == rotStepsTo && moving == 0)  rotON = 0;  // turn off motor if it reaches the set point

        
     
     
    }
}

    //inOutStepsTo = (1600.0 / (360.0 / (T2)));
  //} //else
    //digitalWrite(enPin_rot, HIGH);  // TURN OFF THE MOTOR.
//}
//else
//digitalWrite(enPin_rot, HIGH);  // TURN OFF THE MOTOR.
//}

ISR(TIMER1_COMPB_vect) {  // speed for second motor


  OCR1B += speed_InOut;  // Advance The COMPB Register //8750

  

  if (inOutON == 1) {



    digitalWrite(enPin_InOut, LOW);  // Enable the stepper motor

    pwm2 = !pwm2;
    digitalWrite(stepPin_InOut, pwm2);


    if (pwm2 == 0) {


      if (moving == 0) {
        int state = inOut;
        if (inOutSteps > inOutStepsTo) { inOut = 0; }  // go in
        if (inOutSteps < inOutStepsTo) { inOut = 1; }  // go out
                                                       // if (state!=inOut) inOutSteps+=135;// a change has occured.
        digitalWrite(dirPin_InOut, !inOut);            // Set direction
      }




      if (inOut == 1) inOutSteps = inOutSteps + 1;
      if (inOut == 0) inOutSteps = inOutSteps - 1;



      if (inOutSteps > 5000 && moving == 1) {
        inOut = 0;
        digitalWrite(dirPin_InOut, 1);  // Set direction
      }
      if (inOutSteps < 0 && moving == 1) {

        inOut = 1;
        digitalWrite(dirPin_InOut, 0);  // Set direction
      }





      


      if (inOutSteps == inOutStepsTo && moving == 0) {
        inOutON = 0;
        }
    }
  } //else


}

