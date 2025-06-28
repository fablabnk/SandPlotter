// BY NEWSON'S ELECTRONICS
// The Spirograph Machine can draw any function in the sand
// it's great way to learn the mathematics of art. 
// JULY 10,2024


#include <Adafruit_NeoPixel.h>


int state = 1;                                         // used to determine if inOut motor changes direction
int count = 0;                                         // Count the number of pedels.

int time=0;
int pause_time = 3000;                                 // Time for delay between patturn sequence
byte knobs = 0;
byte drawing=0;
byte clearing=0;
byte clearInOut=0;

int xt=0;

//IR remote code does not need a library NEC protocal.
#define IR_Input 9 // connected to x-limit switch. 
#define Sync_Time 3000
#define One_Time 1000
#define Zero_Time 400
volatile byte Bit_Count = 0;
volatile byte Byte_Count = 0;
volatile byte Byte_Ready = false;
volatile byte IR_Bytes[4];
volatile unsigned long Start_Time = 0;
volatile unsigned long Pulse_Width;
//IR variables

// circle size radius = 100 units
byte pause = 0;
#define enPin_rot 8     // Stepper motor enable pin
#define stepPin_rot 2    // X axis step pin
#define dirPin_rot 5     // X axis direction pin
#define enPin_InOut 8    // Stepper motor enable pin
#define stepPin_InOut 3  // Y axis step pin
#define dirPin_InOut 6   // Y axis direction pin


#define lights_pin 11    //z switch

// Changable variables.
#define rot_total_steps 32000.0     // 16000.0
#define inOut_total_steps 8600.0    // 4300.0
#define desired_scale 100.0


int knob1 = 0;         // POT 1 : LED BRIGHTNESS
int knob2 = 0;         // POT 2 : DRAW SPEED
int brightness = 0;  // Starting brightness for LEDs MAX IS 255
float speed_delay = 1;


//VARIABLES FOR REVERSE KINEMATICS
volatile float r = 0;  // length of extended arm
volatile float x = 0;
volatile float y = 0;  // current location of ball
volatile float x2 = 0;
volatile float y2 = 0;
float xLoc = 0;
float yLoc = 0;
float radAngle = 0;
float rotAngle = 0;


int offset = 0;  // USED TO COUNTER THE GEARS RATIO 10:1
int moving = 0;



int inOutSteps = 0;  /// Y
int inOutStepsTo = 0;
int rotSteps = 0;  //  X
int rotStepsTo = 0;
byte rotON = 0;
byte inOutON = 0;

int ratio_select = 7;


// Variables to store results
float rTo, angleTo;

int pwm = 0;   // ROT MOTOR PWM
int pwm2 = 0;  // INOUT MOTOR PWM

byte inOut = 1;  // 1=OUT, 0=IN
byte ccwCW = 1;  // 0=CCW 1=CW

int speed_InOut = 100;  // speed of second motor
int speed_rot = 100;    // speed of first motor

// change in distance







int star[] = {-11,5,-21,-9,-4,-9,5,-23,10,-6,25,-1,12,9,12,26,-1,16,-16,21,-11,5,-23,7,-42,-19,-11,-18,6,-44,15,-14,44,-4,20,14,20,46,-4,27,-33,37,-23,7,-33,8,-58,-27,-17,-26,7,-61,19,-20,59,-6,25,19,26,62,-7,36,-46,49,-33,8,-41,9,-71,-33,-21,-32,8,-74,23,-24,70,-8,30,22,31,74,-9,43,-56,59,-41,9,-48,11,-83,-39,-25,-37,8,-87,26,-29,82,-10,34,25,36,86,-11,49,-66,69,-48,11,};

int spiral[]={3,0,-7,4,-14,-5,-6,-16,8,-11,7,6,-10,10,-19,-7,-5,-22,15,-11,10,11,-14,13,-23,-11,-2,-26,20,-10,10,17,-19,15,-25,-14,2,-29,25,-7,9,22,-24,15,-27,-19,7,-32,29,-2,7,29,-30,16,-28,-25,13,-35,35,3,4,37,-38,15,-30,-33,21,-38,42,11,-1,46,-48,14,-30,-43,32,-41,49,22,-9,57,-60,10,-29,-54,45,-42,54,35,-19,68,-73,4,-25,-67,59,-41,58,51,-33,77,-84,-5,-19,-79,74,-36,58,68,-48,83,-94,-17,-10,-90,87,-28,54,83,};


//int star[] = {73, -68, 73, -68} ;
int maze[] = {73, -68, 73, -68, 73, -60, 60, -60, 60, -80, 59, -81, 55, -84, 50, -87, 46, -89, 46, -89, 46, -87, 33, -87, 33, -95, 30, -96, 26, -97, 21, -98, 16, -99, 11, -100, 6, -100, 6, -100, 6, -87, 20, -87, 20, -74, 6, -74, 6, -60, 33, -60, 33, -74, 46, -74, 46, -34, 33, -34, 33, -47, 6, -47, 6, -34, 20, -34, 20, -20, 6, -20, 6, -7, 33, -7, 33, -20, 46, -20, 46, -7, 73, -7, 73, -20, 60, -20, 60, -47, 73, -47, 73, -34, 86, -34, 86, -47, 88, -47, 90, -44, 92, -39, 93, -35, 95, -30, 96, -25, 97, -20, 97, -20, 86, -20, 86, -7, 99, -7, 99, -5, 99, 0, 99, 5, 99, 10, 98, 15, 97, 20, 97, 20, 86, 20, 86, 6, 60, 6, 60, 20, 73, 20, 73, 33, 60, 33, 60, 46, 86, 46, 86, 33, 94, 33, 92, 37, 90, 42, 88, 46, 86, 50, 83, 55, 80, 59, 77, 63, 74, 66, 71, 70, 67, 73, 63, 76, 60, 80, 60, 80, 60, 73, 67, 73, 69, 71, 73, 67, 73, 67, 73, 60, 33, 60, 33, 73, 46, 73, 46, 88, 42, 90, 37, 92, 33, 94, 33, 94, 33, 86, 20, 86, 20, 97, 16, 98, 11, 99, 6, 99, 6, 99, 6, 73, 20, 73, 20, 60, 6, 60, 6, 33, 20, 33, 20, 46, 46, 46, 46, 33, 33, 33, 33, 20, 46, 20, 46, 6, 20, 6, 20, 20, 6, 20, 6, 6, -7, 6, -7, 20, -20, 20, -20, 6, -47, 6, -47, 20, -34, 20, -34, 33, -47, 33, -47, 46, -20, 46, -20, 33, -7, 33, -7, 60, -20, 60, -20, 73, -7, 73, -7, 99, -11, 99, -16, 98, -20, 97, -20, 97, -20, 86, -34, 86, -34, 94, -38, 92, -43, 90, -47, 88, -47, 88, -47, 73, -34, 73, -34, 60, -74, 60, -74, 67, -72, 69, -68, 73, -68, 73, -60, 73, -60, 80, -64, 77, -68, 73, -71, 70, -75, 66, -78, 63, -81, 59, -84, 55, -86, 51, -89, 46, -91, 42, -93, 37, -95, 33, -95, 33, -87, 33, -87, 46, -60, 46, -60, 33, -74, 33, -74, 20, -60, 20, -60, 6, -87, 6, -87, 20, -98, 20, -99, 17, -100, 12, -100, 8, -100, 3, -100, -2, -100, -7, -100, -7, -87, -7, -87, -20, -98, -20, -98, -24, -96, -29, -95, -34, -93, -38, -91, -43, -89, -47, -89, -47, -87, -47, -87, -34, -74, -34, -74, -47, -60, -47, -60, -20, -74, -20, -74, -7, -47, -7, -47, -20, -34, -20, -34, -7, -7, -7, -7, -20, -20, -20, -20, -34, -7, -34, -7, -47, -34, -47, -34, -34, -47, -34, -47, -74, -34, -74, -34, -60, -7, -60, -7, -74, -20, -74, -20, -87, -7, -87, -7, -100, -10, -100, -15, -99, -20, -99, -24, -98, -29, -96, -34, -95, -34, -95, -34, -87, -47, -87, -47, -89, -48, -88, -52, -86, -57, -83, -60, -80, -60, -80, -60, -60, -74, -60, -74, -68, -71, -71, -71, -71};


// rot speed:inOut speed numbers 1 to 9 1= fast, 9 = slow
//motor_ratios(9,1);




void spirographWithSquare(float s, float d) // s = Side length of the square, d = Offset from the center to the drawing tip
{
    int numPoints = 300; // Number of points for the spirograph
    float R = 90; // Radius of the fixed circle

    drawing = 1; // Start drawing

    for (int i = 0; i < numPoints; i++) {
        // Calculate the angle for this point
        float angle = 20 * PI * i / numPoints;

        // Calculate the radius of the path traced by the center of the square
        float r = (R - s / 2); // Center of the square will move along a circle of radius (R - s / 2)

        // Position of the square's center
        float cx = r * cos(angle);
        float cy = r * sin(angle);

        // Calculate the angle of rotation of the square
        float rotationAngle = (R / s) * angle;

        // Calculate the position of the drawing tip on the square
        float tx = d * cos(rotationAngle) - (s / 2) * sin(rotationAngle);
        float ty = d * sin(rotationAngle) + (s / 2) * cos(rotationAngle);

        // Offset from the center of the square
        float x = cx + tx;
        float y = cy + ty;

        // Move the drawing arm to the calculated coordinates
        read_pots();
        gotoXY(x, y);
    }

    drawing = 0; // Stop drawing

    // Add a delay before starting the next pattern if needed
    // delay(100);
}

void spirograph (float r,float a)// r=Radius of the rolling circle, a= Distance from the center of the rolling circle
{
//const int radius = 100; // Radius of the circle
int numPoints = 300; // Number of points for the spirograph
float R = 80; // Radius of the fixed circle
  
 
 for (int i = 0; i < numPoints; i++) {
    // Parameters for the spirograph pattern
    // Calculate the angle for this point
   drawing=1;
    float angle = 6 * PI * i / numPoints;

    // Calculate the spirograph coordinates
    int x = (R - r) * cos(angle) + a * cos(((R - r) / r) * angle);
    int y = (R - r) * sin(angle) - a * sin(((R - r) / r) * angle);


// Calculate the epicycloid coordinates
   // int x = (R + r) * cos(angle) - a * cos(((R + r) / r) * angle);
   // int y = (R + r) * sin(angle) - a * sin(((R + r) / r) * angle);


       // Move the drawing arm to the calculated coordinates
        read_pots();
    gotoXY(x, y);
   

  }

drawing=0;
  // Add a delay before starting the next pattern
 // delay(100);

}


void motor_ratios(int r, int t) //r=rotation speed, // t= arm speed
{
  
  //int speed_rot_map[] = { 0, 9, 4, 2, 1, 1,1, 1 };
//int speed_InOut_map[] = { 0, 1, 1, 1, 1,7, 8, 9 };  

  x=0;
  y=0;
   digitalWrite(dirPin_rot, HIGH);  // CW rotation
  read_pots();
   knobs=1;
   //moving=0;
   //drawing=0;
   inOut = 1;  // 1=OUT, 0=IN
 ccwCW = 1;  // 0=CCW 1=CW
  
 rotON = 1;
  inOutON = 1;
  //moving=1;
  //turn on tracking 
  //moveTo(0,0);

  speed_rot = r * speed_delay;
    speed_InOut = t * speed_delay;
     count = 0;
 //display();
 //delay(3000);


while (rotON == 1||inOutON == 1)
{
 read_pots();
   speed_rot = r * speed_delay;
    speed_InOut = t * speed_delay;
     
 display();
 calculate_xy(); // keep track of the location as it moves

 if ((r-t == 8 && count == 34) || 
    (r-t == 7 && count == 30) ||  // Assuming the pattern decreases count by 4
    (r-t == 6 && count == 26) ||  // Adjust the count values as needed
    (r-t == 5 && count == 22) ||
    (r-t == 4 && count == 18) ||
    (r-t == 3 && count == 15) || 
    (r-t == 2 && count == 10) || 
    (r-t == 1 && count == 6)|| 
    (r-t == 0 && count == 23)|| 
    (r-t < 0 && count == 1)) 
{
    rotON = 0;
    inOutON = 0;
    knobs = 0;
    count = 0;
    // moving = 0;
}
  

}

   knobs=0;
   count = 0;


     

}

void calculate_xy()
{
  //if (moving == 1) {
  r = (inOutSteps / inOut_total_steps) * desired_scale;  // changing
  x = r * cos(radAngle);
  y = r * sin(radAngle);
  //}
}



void clear_from_out()
{
  moveTo(0,100);
knobs=1;
ccwCW=0;
digitalWrite(dirPin_rot, ccwCW);

 rotON = 1;
  inOutON = 1;
  while (inOutSteps > 1)
  {
    display();
    read_pots();
    speed_rot= 1 * speed_delay;
speed_InOut=9 * speed_delay;
    calculate_xy();
  }
  knobs=0;
   rotON = 0;
  inOutON = 0;

}


void clear_from_in()
{
 moveTo(0,0);
 motor_ratios(1,9);
}




// HOMING JUST MOVED IN AS FAR AS POSSIBLE NO SENSOR NEEDED
void homing() {
  digitalWrite(enPin_InOut, LOW);   // Enable the stepper motor
  digitalWrite(dirPin_InOut, 1);    // Set direction to go in
  for (int x = 0; x < inOut_total_steps; x++) {  // BLIND - DON'T KNOW HOW FAR TO GO IN GO IN TOTAL STEPS = 0,0
    digitalWrite(stepPin_InOut, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin_InOut, LOW);
    delayMicroseconds(100);
  }
  inOut = 1;
  inOutON = 1;


  digitalWrite(dirPin_InOut, 0);  // Set direction TO OUT
}



void plotShape(int shape[], int size) {
drawing=1;
  for (int t = 0; t < (size / 2); t++)  // size of array div 2.
  {
    // if (button > 0) { return; }
    x2 = shape[t * 2];
    y2 = shape[t * 2 + 1];
    Serial.print(x2);
    Serial.print(",");
    Serial.println(y2);
    gotoXY(x2, y2);  //shift data left and down 50 units to make gride x100 by y100
  }
  drawing=0;// finished drawing patturn
}

// go to a specific point using iterpolation
void gotoXY(float targetX, float targetY) {
  // Calculate the difference between current and target positions
  float dx = targetX - x;
  float dy = targetY - y;

  // Calculate the total distance to travel
  float distance = sqrt(dx * dx + dy * dy);

  // Calculate the number of steps required
  int steps = (int)(distance * 1);  // Adjust stepsPerUnit according to your system

  // Ensure at least one step to move
  if (steps == 0) steps = 1;

  // Calculate the increments per step
  float incrementX = dx / steps;
  float incrementY = dy / steps;

  for (int i = 0; i <= steps; i++) {
    // Calculate the intermediate positions
    float intermediateX = x + incrementX * i;
    float intermediateY = y + incrementY * i;

    // Move the motors to the intermediate positions
    moveTo(intermediateX, intermediateY);
  }

  // Update the current position
  x = targetX;
  y = targetY;
}


// move to a point without interpolation.
void moveTo(float x4, float y4) {

  //digitalWrite(enPin_InOut, HIGH);   // Enable the stepper motor
  x2 = x4;
  y2 = y4;
  reverseKinematics(x4, y4);

  while (rotON == 1 || inOutON == 1) {
    // DO NOTHING!!! wait until hits rot Target location.
    //display();
    read_pots();
    delay(1);
  }
}




void display() {

  if (moving == 1) calculate_xy();
 

  Serial.print(time);
  Serial.print(" \ ");
  Serial.print(count);
  Serial.print(" \ ");
   Serial.print(knobs);
  Serial.print(" \ ");
 
    Serial.print(speed_rot);
  Serial.print(",");
  Serial.print(speed_InOut);
  Serial.print(",");




  Serial.print(offset);
  Serial.print(" *  ");
  Serial.print(inOutON);
  Serial.print(",");
  Serial.print(rotON);
  Serial.print(" *  ");
  

  Serial.print(x);
  Serial.print("x,y");
  Serial.print(y);
  Serial.print(" ");



  Serial.print(rotSteps);
  Serial.print(">");

  Serial.print(rotStepsTo);
  Serial.print(" ");
  Serial.print(inOutSteps);
  Serial.print(">");

  Serial.print(inOutStepsTo);
  // Serial.print(" (");
  //Serial.print(x2);
  //Serial.print(",");
  //Serial.print(y2);
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
  if (rotStepsTo == rot_total_steps) rotStepsTo = 0;
  inOutStepsTo = (rTo / desired_scale) * inOut_total_steps;
  if (inOutStepsTo > inOut_total_steps) inOutStepsTo = inOut_total_steps;  // in case point is outside circle.



  //(turn on the direction )
  if (inOutSteps != inOutStepsTo) inOutON = 1;
  if (rotSteps != rotStepsTo) rotON = 1;



  // Determine how to go IN or OUT?
  if (inOutSteps == inOutStepsTo) { inOutON = 0; }
  if (inOutSteps > inOutStepsTo) {
    inOut = 0;
    digitalWrite(dirPin_InOut, !inOut);
  }  // go in
  if (inOutSteps < inOutStepsTo) {
    inOut = 1;
    digitalWrite(dirPin_InOut, !inOut);
  }  // go out

  // determine if to go CW or CCW
  if ((rotStepsTo > rotSteps && (rotStepsTo - rotSteps < rot_total_steps / 2)) || (rotStepsTo < rotSteps && (rotSteps - rotStepsTo > rot_total_steps / 2))) {
    ccwCW = 0;
    digitalWrite(dirPin_rot, ccwCW);  // Set direction
  } else {
    ccwCW = 1;
    digitalWrite(dirPin_rot, ccwCW);  // Set direction
  }
}

void read_pots() {
  knob1 = analogRead(A0);  // Pot on the left FOR BRIGHTNESS
  knob2 = analogRead(A1);  // Pot on the right FOR DRAWING speed
  brightness = map(knob1, 0, 1023, 255, 0);
  analogWrite(lights_pin, brightness);
  
  
  speed_delay = map(knob2, 0, 1023, 50, 2000);  // MIN SPEED IS 40MS FOR MOTORS TO WORK.
  
  
  
  if (moving== 1) {
     //ratio_select = 0;// same speed for both motors
    speed_rot = speed_delay;
    speed_InOut = speed_delay;
  }

  

if (drawing== 1) {
  speed_delay = map(knob2, 0, 1023, 100, 2000);  // MIN SPEED IS 40MS FOR MOTORS TO WORK.
    speed_rot = speed_delay;
    speed_InOut = speed_delay;
  }




if (knob2 == 1023) pause=1;
///if (knob2 < 1023&&time>2) pause=1;

while (pause == 1) {
  
    digitalWrite(enPin_InOut, HIGH);  // BOTH MOTORS OFF
    digitalWrite(enPin_rot, HIGH);    
    //inOutON=0;//
//rotON=0;

    
    knob1 = analogRead(A0);                // Pot on the left Patturn
    knob2 = analogRead(A1); 
    analogWrite(lights_pin, brightness);
    brightness = map(knob1, 0, 1023, 255, 0);
display();
       delay(100);
    if (time>0) time=time+1;
    

    // turn off pause if turned knob or exceeded pause time. 
    if ((knob2 < 1023&&time==0)||time>pause_time||(knob2 == 1023&&time>1)) 
    {  
        // inOutON=1;//
//rotON=1;
    pause=0;
    time=0;
    digitalWrite(enPin_InOut, LOW);  // BOTH MOTORS ON
    digitalWrite(enPin_rot, LOW);    //
   



    }
 
    
    

}

}


void clear_Left_right()
{
// Clear from left and right
for (int i = -95; i <= 95; i += 10) {
    int xt = sqrt(10000 - (i * i));  // Calculate xt based on circle equation
    gotoXY(xt, i);
    gotoXY(-xt, i); 
    int newY = i + 5;
    if (newY <= 95) {
        // Move to -xt, newY
        gotoXY(-xt, newY);
        // Move back to xt, newY
        gotoXY(xt, newY);
    }
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
  pinMode(stepPin_rot, OUTPUT);
  pinMode(dirPin_rot, OUTPUT);
  pinMode(stepPin_InOut, OUTPUT);
  pinMode(dirPin_InOut, OUTPUT);
  pinMode(enPin_InOut, OUTPUT);
  pinMode(enPin_rot, OUTPUT);
  pinMode(lights_pin, OUTPUT);
  digitalWrite(enPin_InOut, LOW);  // BOTH MOTORS ON
  digitalWrite(enPin_rot, LOW);    //

  Serial.begin(9600);
  Serial.println("sTARting ");
  digitalWrite(dirPin_InOut, 0);  // Set direction
  display();
  homing();

 
  digitalWrite(enPin_InOut, LOW);  // BOTH MOTORS ON
  digitalWrite(enPin_rot, LOW);    //
  Serial.println("READY ");

  rotON = 0;
  inOutON = 0;
  digitalWrite(dirPin_rot, HIGH);  // CW rotation
  display();
}

void loop() {
  read_pots();

// // you can change the order of patterns
// clear_from_in();
// plotShape(maze, sizeof(maze) / sizeof(maze[0]));
// time=1;
// pause=1;
// clear_from_out();
// motor_ratios(4,1); // star ratio
// time=1;
// pause=1;
// clear_Left_right();
// plotShape(spiral, sizeof(spiral) / sizeof(spiral[0]));
// time=1;
// pause=1;
// clear_from_out();
// plotShape(star, sizeof(star) / sizeof(star[0]));
// time=1;
// pause=1;
// clear_from_out();
// motor_ratios(1,1); // 2 loops issues
// time=1;
// pause=1;


  // clear_Left_right();
  homing();
  plotShape(star, sizeof(star) / sizeof(star[0]));
  // plotShape(maze, sizeof(maze) / sizeof(maze[0]));
  // spirograph(15,30); //r,a
  time=1;
  pause=1;

  // CAN ALSO SEND COMMANDS VIA USART
  if (Serial.available() > 0) {
    char command = Serial.read();   // Read the incoming byte
    if (command == 'q') {  // turn on x motor
      rotON = !rotON;
      moving = !moving;
    }
    if (command == 'w') {               // change dir of x motor
      digitalWrite(dirPin_rot, ccwCW);  // Set direction
      ccwCW = !ccwCW;
    }
    if (command == 'e') {  // change dir of x motor
      speed_rot += 50;
    }
    if (command == 'r') {  // change dir of x motor
      speed_rot -= 50;
    }
    if (command == 'a') {  // turn on x motor
      inOutON = !inOutON;
      moving = !moving;
    }
    if (command == 's') {                 // change dir of x motor
      digitalWrite(dirPin_InOut, inOut);  // Set direction
      inOut = !inOut;
    }
    if (command == 'd') {  // change dir of x motor
      speed_InOut -= 50;
    }
    if (command == 'f') {  // change dir of x motor
      speed_InOut += 50;
    }
    if (command == '1') {
      knobs = !knobs;
      inOutON = 1;
      rotON = 1;
    }
    if (command == '2') {
      clear_from_in();
    }
    if (command == '3') {
      clear_from_out();
    }
    if (command == '5') {
      plotShape(star, sizeof(star) / sizeof(star[0]));
    }
    if (command == 'x') {
      plotShape(maze, sizeof(maze) / sizeof(maze[0]));
    }
    if (command == 'c') {
      clear_Left_right();
    }
    if (command == 'v') {
      plotShape(spiral, sizeof(spiral) / sizeof(spiral[0]));
    }
    if (command == 'z') {
      clear_from_in();
      motor_ratios(1,3);
      clear_from_out();
    }
  }
}


//IR SENSOR REMOTE CONTROL
ISR(PCINT0_vect) {
  // When the pin goes HIGH record the pulse start time
  if (digitalRead(IR_Input) == HIGH) {
    Start_Time = micros();
  } else if (Start_Time != 0) {  // Pin went LOW
    // Calculate the pulse width
    Pulse_Width = micros() - Start_Time;
    // Clear the timer
    Start_Time = 0;

    if (Pulse_Width > Sync_Time) {
      // Sync bit
      Bit_Count = 0;
      Byte_Count = 0;
    } else if (Pulse_Width > One_Time) {
      // Data bit = 1
      bitSet(IR_Bytes[Byte_Count], Bit_Count);
      Bit_Count++;
    } else if (Pulse_Width > Zero_Time) {
      // Data bit = 0
      bitClear(IR_Bytes[Byte_Count], Bit_Count);
      Bit_Count++;
    } else {
      Bit_Count = 0;  // Error
    }

    if (Bit_Count == 8) {  // 8 bits to a byte
      Byte_Ready = true;
      Bit_Count = 0;
      Byte_Count++;
    }

    if (Byte_Count == 4) {
      Byte_Count = 0;

      // Check for valid header
      if ((IR_Bytes[0] == 0x00) && (IR_Bytes[1] == 0xFF)) {
        // Decode command byte
        byte button = IR_Bytes[2];
        // Serial.print("button="); // print the value of the button if you use a different controller.
        // Serial.println(button);
        if (button == 24) brightness = brightness + 10;  //up
        if (button == 82) brightness = brightness - 10;  // down
        if (button == 90) pause = !pause;                // right
        if (button == 8) pause = !pause;                 // left

        if (button == 28) pause = !pause;                // ok button
        if (button == 22) brightness = brightness - 10;  // *
        if (button == 13) brightness = brightness + 10;  // #

        // Numbers
      
        if (button == 21) pause = !pause;    // 8
        if (button == 9) pause = !pause;     // 9
        if (button == 25) pause = !pause;    // 0
      }
    }
  }
}



// ISR for the rotational motor
ISR(TIMER1_COMPA_vect) {
  OCR1A += speed_rot;  // Advance the COMPA register



  if (rotON == 1 && pause == 0) {
    pwm = !pwm;
    digitalWrite(stepPin_rot, pwm);  // ONE STEP OF ROT MOTOR

    //uncloment later
    if (offset == 0 && inOutON == 0 && rotON == 1) {  // If only Rot motor is on composinate for inout motor direction is opposite rot direction
      digitalWrite(stepPin_InOut, HIGH);
      digitalWrite(dirPin_InOut, !ccwCW);  // Set direction opposite rot motor
    }



    if (pwm == 0) {
      offset += 1;  // track each step of rot

      if (ccwCW == 1) {
        rotSteps -= 1;
        //digitalWrite(dirPin_rot, HIGH);
        if (offset >= 10) {  //after 10 steps compinstate for inout motor
          offset = 0;
          if (inOutON == 1 && rotON == 1) inOutSteps -= 1;
          if (inOutON == 0 && rotON == 1) digitalWrite(stepPin_InOut, LOW);
        }

      } else {
        rotSteps += 1;
        //  digitalWrite(dirPin_rot, LOW);
        if (offset >= 10) {
          offset = 0;
          if (inOutON == 1 && rotON == 1) inOutSteps += 1;
          if (inOutON == 0 && rotON == 1) digitalWrite(stepPin_InOut, LOW);
        }
      }



      // Adjust the direction if limits are reached (note sure if needed?)
      if (inOutSteps > inOut_total_steps) {
        inOut = 0;
        digitalWrite(dirPin_InOut, HIGH);  // Set direction
      } else if (inOutSteps < 0) {
        inOut = 1;
        digitalWrite(dirPin_InOut, LOW);  // Set direction
      }


      // Wrap-around conditions for rotational steps
      if (rotSteps > rot_total_steps) rotSteps = 0;
      else if (rotSteps < 0) rotSteps = rot_total_steps - 1;

      radAngle = (rotSteps / rot_total_steps) * (2 * PI);  // Update angles
      rotAngle = (rotSteps / rot_total_steps) * 360.0;




      if (rotSteps == rotStepsTo && knobs == 0&&moving==0&&clearing==0) { rotON = 0; }  // turn off motor if it reaches the set point

      if (inOutSteps == inOutStepsTo && knobs == 0&&moving==0&&clearing==0) inOutON = 0;
    }
  }
}

// ISR for the in/out motor
ISR(TIMER1_COMPB_vect) {
  OCR1B += speed_InOut;  // Advance the COMPB register

  if (inOutON == 1&&pause==0) {
    pwm2 = !pwm2;


    if (inOut == 0) digitalWrite(dirPin_InOut, HIGH);  // Set direction
    if (inOut == 1) digitalWrite(dirPin_InOut, LOW);   // Set direction

    digitalWrite(stepPin_InOut, pwm2);







    if (pwm2 == 0) {
      // Set the direction based on the target steps

      // Update in/out steps based on the direction
      if (inOut == 1) inOutSteps++;
      else inOutSteps--;

      if (inOutSteps == inOutStepsTo && knobs == 0&&moving==0&&clearing==0) { inOutON = 0; }  //offset=0;


      // Adjust the direction if limits are reached
      if (inOutSteps > inOut_total_steps) {
        inOut = 0;
        digitalWrite(dirPin_InOut, HIGH);  // Set direction
      } else if (inOutSteps < 0) {
        inOut = 1;
        digitalWrite(dirPin_InOut, LOW);  // Set direction
      }
      if (knobs == 1) {

        if (state != inOut) count += 1;
        state = inOut;
      }  


        
    }
  }
}