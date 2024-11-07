#include "MotorDefs.h"

#include <AccelStepper.h>

#include "pid.h"

// -------------- Global Definitions -----------------
// Motor Angle Struct
struct motorAngles motorAnglesObj;

// Global Ball Position
float x_ball_pixel;
float y_ball_pixel;

// PID Related Definitions
int output = 0; //output from PID algorithm
long prevT = 0; //previous time
float errorPrev = 0; //previous error
float integral = 0; //integral term

// Calibration Related Consts
float theta0 = -2.9;

// I2C Pins
const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int ledPin = 13; // *** Change LED Pin here
int LED_Byte = 0;

AccelStepper stepper1(AccelStepper::DRIVER, stepPinStepper1, dirPinStepper1); //create instance of stepper
AccelStepper stepper2(AccelStepper::DRIVER, stepPinStepper2, dirPinStepper2); //create instance of stepper
AccelStepper stepper3(AccelStepper::DRIVER, stepPinStepper3, dirPinStepper3); //create instance of stepper

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

  stepper3.disableOutputs(); //disable outputs initially
  stepper3.setMaxSpeed(20000);
  stepper3.setCurrentPosition(0); //zero current stepper position
  stepper3.enableOutputs(); //enable outputs for motor

  calibrate(); 

  // Set up I2C
  // Arduino joins I2C bus as slave with address 8
  Wire.begin(0x8);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(getBallCoords);
}

void receiveEvent(int[] howMany) {
  LED_Byte = Wire.read(); // receive byte as an integer
  digitalWrite(ledPin, LED_Byte); // turn on/off LED based on byte information
  Serial.println(LED_Byte);
  
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

void getBallCoords(int[] howMany)
{
  // Function that executes whenever data is received from master device, the Pi 5
  xCoords = Wire.read(); // receive byte as an integer
  Serial.println(xCoords);

  // Set the variables below
  // x_ball_pixel
  // y_ball_pixel
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

  // float theta_m;

  // motorAngles.motor1 = 
  // motorAngles.motor2 = 
  // motorAngles.motor3 = 
}

float PID_Helper(float target_pos, float curr_xy_ball_pos) {
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

void PID(){
  // Perform PID Calculations
  float target_x_pos = 960;
  float theta_x_output = PID_Helper(target_x_pos, x_ball_pixel);

  float target_y_pos = 540;
  float theta_y_output = PID_Helper(target_y_pos, y_ball_pixel);

  // Perform inverse kinematics eqns to find motor1, motor2, and motor3 angle vals.
  inverseKinematics(theta_x_output, theta_y_output);
}

// Main function to be run in the arduino
void loop() {
  // Call PID helpers here and compute differences 
  // PID();

  //stepper1.moveTo(angleToStep(720+360));
  //delay(1000);
  //stepper1.runToPosition();
}