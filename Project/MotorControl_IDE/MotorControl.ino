#include <AccelStepper.h>
#include <MultiStepper.h>

// ------------Motor Angle Definitions----------------------
const int dirPinStepper1 = 8; //direction Pin (DIR+)
const int stepPinStepper1 = 10; //pulse Pin (PUL+)
const int enPinStepper1 = 9; //enable Pin (ENA+)

const int dirPinStepper2 = 11; //direction Pin (DIR+))
const int stepPinStepper2 = 13; //pulse Pin (PUL+)
const int enPinStepper2 = 12; //enable Pin (ENA+)

const int dirPinStepper3 = 5; //direction Pin (DIR+))
const int stepPinStepper3 = 7; //pulse Pin (PUL+)
const int enPinStepper3 = 6; //enable Pin (ENA+)

// --------------PID Constants----------------------
float kp = 1; //*1
float kd = 0.025; //*2
float ki = 0.5; //*3

// Define maximum and minimum integral limits
const float MAX_INTEGRAL = 100.0;
const float MIN_INTEGRAL = -100.0;

long prevT = 0; //previous time
float errorPrev = 0; //previous error
float integral = 0; //integral term

// -------------- Global Definitions -----------------
float motorAngles[3] = {0};

// Global Ball Position
float x_ball_pixel;
float y_ball_pixel;

// --------------Platform Constants-----------------
// Everything in mm
int link1_length = 45;
int link2_length = 75;
int motor_radius = 75;
int platform_radius = 90;

float z_initial = 66.77; // from CAD
int MAX_MOTOR = 40;// 43.9 in CAD
int MIN_MOTOR = -20; //-25.7 in CAD

// --------------Calibration Constants-----------------
float theta0 = -2.9;

// --------------I2C Constants-----------------
const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int ledPin = 13; // *** Change LED Pin here
int LED_Byte = 0;

// --------------Stepper Instances-----------------
AccelStepper stepper1(AccelStepper::DRIVER, stepPinStepper1, dirPinStepper1); //create instance of stepper
AccelStepper stepper2(AccelStepper::DRIVER, stepPinStepper2, dirPinStepper2); //create instance of stepper
AccelStepper stepper3(AccelStepper::DRIVER, stepPinStepper3, dirPinStepper3); //create instance of stepper
MultiStepper steppers;

// -------------Setup-----------------
void setup() {
  Serial.begin(9600);

  stepper1.disableOutputs(); //disable outputs initially
  stepper1.setMaxSpeed(10000);
  stepper1.setCurrentPosition(0); //zero current stepper position
  stepper1.enableOutputs(); //enable outputs for motor
  stepper1.setAcceleration(3000);

  stepper2.disableOutputs(); //disable outputs initially
  stepper2.setMaxSpeed(20000);
  stepper2.setCurrentPosition(0); //zero current stepper position
  stepper2.enableOutputs(); //enable outputs for motor
  stepper2.setAcceleration(3000);

  stepper3.disableOutputs(); //disable outputs initially
  stepper3.setMaxSpeed(20000);
  stepper3.setCurrentPosition(0); //zero current stepper position
  stepper3.enableOutputs(); //enable outputs for motor
  stepper3.setAcceleration(3000);

  calibrate(); 

  // Set up I2C
  // Arduino joins I2C bus as slave with address 8
  //  Wire.begin(0x8);
  
  // Call receiveEvent function when data received                
  //  Wire.onReceive(getBallCoords);
}

//void receiveEvent(int[] howMany) {
//  LED_Byte = Wire.read(); // receive byte as an integer
//  digitalWrite(ledPin, LED_Byte); // turn on/off LED based on byte information
//  Serial.println(LED_Byte);
//  
//}

//void getBallCoords(int[] howMany)
//{
//  // Function that executes whenever data is received from master device, the Pi 5
//  xCoords = Wire.read(); // receive byte as an integer
//  Serial.println(xCoords);
//
//  // Set the variables below
//  // x_ball_pixel
//  // y_ball_pixel
//}

void loop() {

  //PID to center of board
  PID(960,540);

  stepper1.moveTo(angleToStep(motorAngles[0]));
  stepper1.moveTo(angleToStep(motorAngles[1]));
  stepper1.moveTo(angleToStep(motorAngles[2]));
  
  stepper1.run();
  stepper2.run();
  stepper3.run();
}

int angleToStep(float angle){
  return int(trunc(angle/360 * 3200));
}

void calibrate(){
  // Platform to be pushed all the way down, angle = -62.78 from CAD
  // Move from theta = -62.78 to theta0
  stepper1.move(angleToStep(62.78+theta0));
  stepper2.move(angleToStep(62.78+theta0));
  stepper3.move(angleToStep(62.78+theta0));

  stepper1.runToPosition();
  stepper2.runToPosition();
  stepper3.runToPosition();
  
  // reset stepper position to be at 
  stepper1.setCurrentPosition(angleToStep(theta0));
  stepper2.setCurrentPosition(angleToStep(theta0));
  stepper3.setCurrentPosition(angleToStep(theta0));
}

void inverseKinematics(float theta_X, float theta_Y)
{
  // Apply eqns and set global motor angles
  float z1_change = (platform_radius*(sin(30))*(sin(theta_Y))) + (platform_radius*(sin(60))*(sin(theta_X)));
  float z2_change = (platform_radius*(sin(30))*(sin(theta_Y))) - (platform_radius*(sin(60))*(sin(theta_X)));
  float z3_change = -(platform_radius*(sin(theta_Y)));

  float z1_final = z_initial + z1_change;
  float z2_final = z_initial + z2_change;
  float z3_final = z_initial + z3_change;

  float k = platform_radius - motor_radius;

  float n1 = sqrt(pow(z1_final, 2) + pow(k, 2));
  float n2 = sqrt(pow(z2_final, 2) + pow(k, 2));
  float n3 = sqrt(pow(z3_final, 2) + pow(k, 2));

  // Motor 1
  float num1 = (((pow(link1_length, 2) + pow(n1, 2) - pow(link2_length, 2)))/(2*n1*link1_length));
  motorAngles[0] = asin((num1)) - atan(k/z1_final);

  // Motor 2
  float num2 = (((pow(link1_length, 2) + pow(n2, 2) - pow(link2_length, 2)))/(2*n2*link1_length));
  motorAngles[1] = asin((num2)) - atan(k/z2_final);

  // Motor 3
  float num3 = (((pow(link1_length, 2) + pow(n3, 2) - pow(link2_length, 2)))/(2*n3*link1_length));
  motorAngles[2] = asin((num3)) - atan(k/z3_final);

  // saturate motor angles between bounds
  for (int i=0; i<=2, i++){
    if (motorAngles[i]>MAX_MOTOR) motorAngles[i] = MAX_MOTOR;
    if (motorAngles[i]<MIN_MOTOR) motorAngles[i] = MIN_MOTOR;
  }
}

float PID_Helper(float target_pos, float curr_pos) {
  // Find time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //determine change in time
  prevT = currT; //reset current time

  // Note: Ball Position is in pixels
  int error = target_pos - curr_xy_ball_pos;
  integral = integral + error*deltaT;
  float derivative = (error - errorPrev)/(deltaT);
  errorPrev = error;
 
  // Clamp the integral term
  if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
  if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

  // Control signal
  float output = kp*error + kd*derivative + ki*integral;

  // The outputs here are the angles we need to perform inv kinematics on (theta_x, theta_y)
  return output;
}

void PID(int target_x, int target_y){
  // Perform PID Calculations
  
  float theta_x_output = PID_Helper(target_x, x_ball_pixel);

  float theta_y_output = PID_Helper(target_y, y_ball_pixel);

  // Perform inverse kinematics eqns to find motor1, motor2, and motor3 angle vals.
  inverseKinematics(theta_x_output, theta_y_output);
}
