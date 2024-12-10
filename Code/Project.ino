/*
MTE 380 Stewart Platform
Krish Vijayan, Julian Yam, Rijin Muralidharan, Om Patel
December 2024
*/

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>

// ------------Motor Angle Definitions----------------------
const int dirPinStepper2 = 11; //direction Pin (DIR+)
const int stepPinStepper2 = 13; //pulse Pin (PUL+)
const int enPinStepper2 = 12; //enable Pin (ENA+)

const int dirPinStepper1 = 8; //direction Pin (DIR+))
const int stepPinStepper1 = 10; //pulse Pin (PUL+)
const int enPinStepper1 = 9; //enable Pin (ENA+)

const int dirPinStepper3 = 5; //direction Pin (DIR+))
const int stepPinStepper3 = 7; //pulse Pin (PUL+)
const int enPinStepper3 = 6; //enable Pin (ENA+)

const int buttonpin = 4;

// --------------PID Constants----------------------
float kp = 0.020; //0.020
float kd = 0.00035;//0.00035;
float ki = 0.006; //0.006

const int SCALE = 1.23;

//curr best
// float kp = 0.020;//0.005; //*1 // MUCH HIGHER
// float kd = 0.0004;//06; //*2 0005
// float ki = 0.006; //*3


// float kp = 0.0;//0.005; //*1 // MUCH HIGHER
// float kd = 0.0;//06; //*2
// float ki = 0.007; //*3
// float kp = 0.5;//0.068; //*1 // MUCH HIGHER
// float kd = 0;//06; //*2
// float ki = 0; //*3

// Define maximum and minimum integral limits
const float MAX_INTEGRAL = 150.0;
const float MIN_INTEGRAL = -150.0;

long prevT_x = 0; //previous time
float errorPrev_x = 0; //previous error
float integral_x = 0; //integral term

long prevT_y = 0; //previous time
float errorPrev_y = 0; //previous error
float integral_y = 0; //integral term

// -------------- Global Definitions -----------------
float motorAngles[3] = {0,0,0};
float Z[3] = {0,0,0};
float num[3] = {0,0,0};

// Global Ball Position
float x_ball_pixel = 305;
float y_ball_pixel = 225;

// --------------Platform Constants-----------------
// Everything in mm
int link1_length = 45;
int link2_length = 75;
int motor_radius = 75;
int platform_radius = 90;

int MAX_HEIGHT = 100;//  ~119 in CAD
int MIN_HEIGHT = 60; //49.98

// --------------Calibration Constants-----------------
float theta_0 = 15;
float z_0 = 81.04; // from CAD

// --------------I2C Constants-----------------
const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int ledPin = 13; // *** Change LED Pin here
int LED_Byte = 0;

// -------------Recieving Coordinates----------
int byteCounter = 0;  // Track which byte we're reading
int x_high = 0, x_low = 0, y_high = 0, y_low = 0;

// -------------Kp & Kd Scaling------------------


const int CENTER[2] = {300,240};
const int MARGIN = 50;

// --------------Stepper Instances-----------------
AccelStepper stepper1(AccelStepper::DRIVER, stepPinStepper1, dirPinStepper1); //create instance of stepper
AccelStepper stepper2(AccelStepper::DRIVER, stepPinStepper2, dirPinStepper2); //create instance of stepper
AccelStepper stepper3(AccelStepper::DRIVER, stepPinStepper3, dirPinStepper3); //create instance of stepper
MultiStepper steppers;

// -------------Setup-----------------
void setup() {
  Serial.begin(9600);
  
  // receiveEvent Setup
  Wire.begin(0x8);
  Wire.onReceive(receiveEvent);
  pinMode(SDA_Pin, INPUT);
  pinMode(SCL_Pin, INPUT);

  // Start button setup
  pinMode(buttonpin, INPUT_PULLUP);

  stepper1.disableOutputs(); //disable outputs initially
  stepper1.setMaxSpeed(17000);
  stepper1.setCurrentPosition(0); //zero current stepper position
  stepper1.enableOutputs(); //enable outputs for motor
  stepper1.setAcceleration(8000);

  stepper2.disableOutputs(); //disable outputs initially
  stepper2.setMaxSpeed(17000);
  stepper2.setCurrentPosition(0); //zero current stepper position
  stepper2.enableOutputs(); //enable outputs for motor
  stepper2.setAcceleration(8000);

  stepper3.disableOutputs(); //disable outputs initially
  stepper3.setMaxSpeed(17000);
  stepper3.setCurrentPosition(0); //zero current stepper position
  stepper3.enableOutputs(); //enable outputs for motor
  stepper3.setAcceleration(8000);

  while (digitalRead(buttonpin)==LOW){

  }

  calibrate(); 

  delay(500);
  //LIN_inverseKinematics(45,45);
}

void loop() {  

  PID(CENTER[0], CENTER[1]);
  
  stepper1.moveTo(angleToStep(motorAngles[0]));
  stepper2.moveTo(angleToStep(motorAngles[1]));
  stepper3.moveTo(angleToStep(motorAngles[2]));

  while ((stepper1.distanceToGo() != 0) || (stepper2.distanceToGo() != 0) || (stepper3.distanceToGo() != 0))
  {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  
}

int angleToStep(float angle){
  return int(trunc(angle/360 * 3200));
}

void calibrate(){
  // Platform to be pushed all the way down, initial angle=-27.46 from CAD
  // Move from theta = -27.46 to theta_0
  stepper1.move(angleToStep(27.46+theta_0));
  stepper2.move(angleToStep(27.46+theta_0));
  stepper3.move(angleToStep(27.46+theta_0));

  stepper1.runToPosition();
  stepper2.runToPosition();
  stepper3.runToPosition();
  
  // reset stepper position to be 15 degrees at neutral position
  stepper1.setCurrentPosition(angleToStep(theta_0));
  stepper2.setCurrentPosition(angleToStep(theta_0));
  stepper3.setCurrentPosition(angleToStep(theta_0));
}

void LIN_inverseKinematics(float theta_Y, float theta_X)
{
  theta_X *= (3.1416/180);
  theta_Y *= -3.1416/180;

  float z1_change = (platform_radius*sqrt(3)/2*theta_X) + (platform_radius/2*theta_Y);

  float z2_change = -((platform_radius*theta_Y));
  
  float z3_change = ((platform_radius/2*theta_Y)) -(platform_radius*sqrt(3)/2*theta_X);

  Z[0] = z_0 + z1_change;
  Z[1] = z_0 + z2_change;
  Z[2] = z_0 + z3_change;

  //clamp z height to avoid weird angles for theta
  for(int i=0;i<=2;i++){
    if (Z[i]>MAX_HEIGHT) Z[i] = MAX_HEIGHT;
    if (Z[i]<MIN_HEIGHT) Z[i] = MIN_HEIGHT;
  }

  //Serial.println("final angles---");

  for (int i=0; i<=2; i++){
    motorAngles[i] = -81.2425565391 + 1.18759328456*Z[i];
  }
  
}

float PID_Helper_y(float target_pos_y, float curr_pos_y) {
  // Find time difference
  long currT_y = micros();
  float deltaT_y = ((float) (currT_y - prevT_y))/( 1.0e6 ); //determine change in time
  prevT_y = currT_y; //reset current time

  float error_y = target_pos_y - curr_pos_y;
  integral_y = integral_y + error_y*deltaT_y;
  float derivative_y = (error_y - errorPrev_y)/(deltaT_y);
  errorPrev_y = error_y;
 
  // Clamp the integral term
  if (integral_y > MAX_INTEGRAL) integral_y = MAX_INTEGRAL;
  if (integral_y < MIN_INTEGRAL) integral_y = MIN_INTEGRAL;

  float output_y;

    if ((y_ball_pixel < CENTER[0] - MARGIN || y_ball_pixel > CENTER[0] - MARGIN) || abs(derivative_y)>700 ){
      output_y = kp*SCALE*error_y + kd*SCALE*derivative_y + ki*SCALE*integral_y;
    }else{
      output_y = kp*error_y + kd*derivative_y + ki*integral_y;
    }

  return output_y;
}

float PID_Helper_x(float target_pos_x, float curr_pos_x) {
  // Find time difference
  long currT_x = micros();
  float deltaT_x = ((float) (currT_x - prevT_x))/( 1.0e6 ); //determine change in time  Serial.print(" PREV D ");

  prevT_x = currT_x; //reset current time

  float error_x = target_pos_x - curr_pos_x;
  integral_x += error_x*deltaT_x;
  float derivative_x = (error_x - errorPrev_x)/(deltaT_x);

  errorPrev_x = error_x;

  if (integral_x > MAX_INTEGRAL) integral_x = MAX_INTEGRAL;
  if (integral_x < MIN_INTEGRAL) integral_x = MIN_INTEGRAL;

  float output_x;
  if ((x_ball_pixel < CENTER[0] - MARGIN || x_ball_pixel > CENTER[0] - MARGIN || abs(derivative_x)>700 )){
        output_x = kp*SCALE*error_x + kd*SCALE*derivative_x + ki*SCALE*integral_x;
    }else{
       output_x = kp*error_x + kd*derivative_x + ki*integral_x;
    }

  return output_x;
}

void PID(int target_x, int target_y){
  // Perform PID Calculations
  
  //Serial.print("X:  ");
  float theta_x_output = PID_Helper_x(target_x, x_ball_pixel);

  //Serial.print("               Y:  ");
  float theta_y_output = PID_Helper_y(target_y, y_ball_pixel);

  // Perform inverse kinematics eqns to find motor1, motor2, and motor3 angle vals.
  LIN_inverseKinematics(theta_x_output, theta_y_output);
}


void receiveEvent(int bytes) {
    while (Wire.available()) {
        if (byteCounter == 0) {
            x_high = Wire.read();  // First byte is the high byte of x
        } else if (byteCounter == 1) {
            x_low = Wire.read();   // Second byte is the low byte of x
            x_ball_pixel = (x_high << 8) | x_low;  // Combine high and low bytes into x

        } else if (byteCounter == 2) {
            y_high = Wire.read();  // Third byte is the high byte of y
        } else if (byteCounter == 3) {
            y_low = Wire.read();   // Fourth byte is the low byte of y
            y_ball_pixel = (y_high << 8) | y_low;  // Combine high and low bytes into y

            byteCounter = -1;
        }
        byteCounter++;

    }
}