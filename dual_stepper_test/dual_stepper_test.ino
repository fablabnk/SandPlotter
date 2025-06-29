// Stepper A pins
const int stepPinA = 2;
const int dirPinA = 5;
const int enablePinA = 8;

// Stepper B pins
const int stepPinB = 3;
const int dirPinB = 6;
const int enablePinB = 8;

bool dirStateA = LOW;
bool dirStateB = LOW;

void setup() {
  pinMode(stepPinA, OUTPUT);
  pinMode(dirPinA, OUTPUT);
  pinMode(enablePinA, OUTPUT);

  pinMode(stepPinB, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(enablePinB, OUTPUT);

  digitalWrite(enablePinA, LOW); // Enable Stepper A
  digitalWrite(enablePinB, LOW); // Enable Stepper B
}

void stepperSequence(int stepPin, int dirPin, int enablePin, bool &dirState) {
  // Set direction to clockwise
  digitalWrite(dirPin, HIGH);

  // Rotate the motor for 200 steps
  for (int i = 0; i < 200; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  delay(1000);

  // Set direction to counter-clockwise
  digitalWrite(dirPin, dirState);

  // Rotate the motor in the opposite direction
  for (int i = 0; i < 200; i++) {
    digitalWrite(enablePin, LOW); // Enable the driver (LOW is enabled)
    delayMicroseconds(500);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    digitalWrite(enablePin, HIGH); // Disable the driver
    delayMicroseconds(500);
  }
  dirState = !dirState;

  delay(1000);
}

void loop() {
  // Alternate sequence between Stepper A and Stepper B
  stepperSequence(stepPinA, dirPinA, enablePinA, dirStateA);
  stepperSequence(stepPinB, dirPinB, enablePinB, dirStateB);
}
