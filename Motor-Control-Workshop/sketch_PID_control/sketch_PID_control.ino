#include <AccelStepper.h>

const int dirPinStepper1 = 2; //direction Pin (DIR+)
const int stepPinStepper1 = 3; //pulse Pin (PUL+)
const int enPinStepper1 = 4; //enable Pin (ENA+)

const int dirPinStepper2 = 6; //direction Pin (DIR+))
const int stepPinStepper2 = 7; //pulse Pin (PUL+)
const int enPinStepper2 = 5; //enable Pin (ENA+)

const int dirPinStepper3 = 9; //direction Pin (DIR+))
const int stepPinStepper3 = 8; //pulse Pin (PUL+)
const int enPinStepper3 = 10; //enable Pin (ENA+)

// Motor Driver 1
// Pinouts (D3 (PUL+), D4 (ENA+), D2 (DIR+))

// Motor Driver 2
// Pinouts (D5 (ENA+), D6 (DIR+), D7 (PUL+))

// Motor Driver 3
// Pinouts (D8 (PUL+), D9 (DIR+), D10 (ENA+))


// PID constants (students can edit these to adjust accuracy)
float kp = 1; //*1
float kd = 0.025; //*2
float ki = 0.5; //*3
int output = 0; //output from PID algorithm

AccelStepper stepper1(AccelStepper::DRIVER, stepPinStepper1, dirPinStepper1); //create instance of stepper
AccelStepper stepper2(AccelStepper::DRIVER, stepPinStepper2, dirPinStepper2); //create instance of stepper
AccelStepper stepper3(AccelStepper::DRIVER, stepPinStepper3, dirPinStepper3); //create instance of stepper

long prevT = 0; //previous time
float errorPrev = 0; //previous error

// Define maximum and minimum integral limits
const float MAX_INTEGRAL = 100.0; //maximum integral limit
const float MIN_INTEGRAL = -100.0; //minimum integral limit

float integral = 0; //integral term

void setup() {
  Serial.begin(9600);
  stepper1.disableOutputs(); //disable outputs initially
  stepper1.setMaxSpeed(10000);
  stepper1.setCurrentPosition(0); //zero current stepper position
  stepper1.enableOutputs(); //enable outputs for motor
  stepper1.setAcceleration(3000);
  // pinMode(2,OUTPUT);
  // pinMode(3,OUTPUT);
  // digitalWrite(2,HIGH);

  stepper2.disableOutputs(); //disable outputs initially
  stepper2.setMaxSpeed(20000);
  stepper2.setCurrentPosition(0); //zero current stepper position
  stepper2.enableOutputs(); //enable outputs for motor

  stepper3.disableOutputs(); //disable outputs initially
  stepper3.setMaxSpeed(20000);
  stepper3.setCurrentPosition(0); //zero current stepper position
  stepper3.enableOutputs(); //enable outputs for motor


        
}

void loop() {
  PID();
  // digitalWrite(3,LOW);
  // digitalWrite(3,HIGH);
  // delayMicroseconds(60);
}

float angleToStep(float angle){
  return angle/360 * 3200;
}

void PID() {

  float target = 90.0 * 1023.0 / 270; // *4 This is the target step we wish to achieve converting from degrees to POT ticks.
  target = constrain(target, 0, 1023); //contrains target to the min/max POT ticks
  Serial.print("target "); //prints out the target
  Serial.print(target); 
  Serial.print(",");

  // Find time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //determine change in time
  prevT = currT; //reset current time

  
  Serial.print("potenValue "); //print out the POT value
  Serial.print(analogRead(A0)); 
  Serial.print(",");

  // PID calculation
  int error = target - analogRead(A0);
  integral = integral + error*deltaT;
  float derivative = (error - errorPrev)/(deltaT);
  errorPrev = error;
 
  // Clamp the integral term
  if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
  if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

  // Control signal
  float output = kp*error + kd*derivative + ki*integral; //PID output (In POT ticks)
  Serial.print("output "); //prints output
  Serial.print(output); 
  Serial.print(",");

  float stepperTarget = round(((((output * 270) / 1023) * 3200) / 360)); //converts POT ticks to steps as motor is set to 3200 steps/revolution
  stepperTarget = (constrain(stepperTarget, -2400, 2400)); //clamps error down too maximum movable steps based on POT
  Serial.print("stepperTarget "); //prints motor's target steps
  Serial.println(stepperTarget);
//  stepper.move(stepperTarget);

  // Moves motors for a certain time before repeating PID calculations
  long currT2 = millis();
  while (1) { // *5 the period of motor movement can be adjusted
//      stepper1.setSpeed(6400); //*6 sets motor speed
//      stepper1.runSpeed(); //steps the motor 
//
//      stepper2.setSpeed(6400); //*6 sets motor speed
//      stepper2.runSpeed(); //steps the motor 
//      
//      stepper3.setSpeed(6400); //*6 sets motor speed
//      stepper3.runSpeed(); //steps the motor 
        stepper1.moveTo(angleToStep(720+360));
        delay(1000);

        stepper1.runToPosition();
        
 }
}

// Motor Driver 1
// Pinouts (D3 (PUL+), D4 (ENA+), D2 (DIR+))

// Motor Driver 2
// Pinouts (D5 (ENA+), D6 (DIR+), D7 (PUL+))

// Motor Driver 3
// Pinouts (D8 (PUL+), D9 (DIR+), D10 (ENA+))
