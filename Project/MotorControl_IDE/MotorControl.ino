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
float motorAngles[3] = {0,0,0};
float Z[3] = {0,0,0};
float num[3] = {0,0,0};

// Global Ball Position
float x_ball_pixel;
float y_ball_pixel;

// --------------Platform Constants-----------------
// Everything in mm
int link1_length = 45;
int link2_length = 75;
int motor_radius = 75;
int platform_radius = 90;

int MAX_HEIGHT = 115;//  ~119 in CAD
int MIN_HEIGHT = 52; //49.98

// --------------Calibration Constants-----------------
float theta_0 = 15;
float z_0 = 81.04; // from CAD

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

  delay(500);
  LIN_inverseKinematics(45,45);
  
}

void loop() {


  //PID to center of board
  //PID(960,540);
  
  stepper1.moveTo(angleToStep(motorAngles[0]));
  stepper2.moveTo(angleToStep(motorAngles[1]));
  stepper3.moveTo(angleToStep(motorAngles[2]));

  stepper1.run();
  stepper2.run();
  stepper3.run();
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

void inverseKinematics(float theta_Y, float theta_X)
{
  theta_X *= 3.1416/180;
  theta_Y *= 3.1416/180;
  //Serial.println("x---");
  //Serial.println(theta_X);
  //Serial.println("y---");
  //Serial.println(theta_Y);

  // Apply eqns and set global motor angles
  float z1_change = (platform_radius/2*(sin(theta_Y))) + (platform_radius*sqrt(3)/2*sin(theta_X));
  float z2_change = (platform_radius/2*(sin(theta_Y))) - (platform_radius*sqrt(3)/2*sin(theta_X));
  float z3_change = -(platform_radius*(sin(theta_Y)));
  
  //Serial.println("z1_change---");
  //Serial.println(z1_change);
  //  Serial.println("z2_change---");
  //Serial.println(z2_change);
  //  Serial.println("z3_change---");
  //Serial.println(z3_change);

  Z[0] = z_0 + z1_change;
  Z[1] = z_0 + z2_change;
  Z[2] = z_0 + z3_change;

  //clamp z height to avoid weird angles for theta
  for(int i=0;i<=2;i++){
    if (Z[i]>MAX_HEIGHT) Z[i] = MAX_HEIGHT;
    if (Z[i]<MIN_HEIGHT) Z[i] = MIN_HEIGHT;
  }

  //Serial.println("fin---");
  //Serial.println(Z[2]);

  float k = platform_radius - motor_radius;
  //Serial.println("k---");
  //Serial.println(k);

  float n1 = sqrt(pow(Z[0], 2) + pow(k, 2));
  float n2 = sqrt(pow(Z[1], 2) + pow(k, 2));
  float n3 = sqrt(pow(Z[2], 2) + pow(k, 2));
  
  //Serial.println("n3---");
  //Serial.println(n3);

  // Motor 1
  num[0] = (((pow(link1_length, 2) + pow(n1, 2) - pow(link2_length, 2)))/(2*n1*link1_length));
  //Serial.println("angle1---");
  //Serial.println(num1);
  //Serial.println(atan(k/z1_final));

  // Motor 2
  num[1] = (((pow(link1_length, 2) + pow(n2, 2) - pow(link2_length, 2)))/(2*n2*link1_length));
  //Serial.println("angle2---");
  //Serial.println(num2);
  //Serial.println(atan(k/z2_final));

  // Motor 3
  num[2] = (((pow(link1_length, 2) + pow(n3, 2) - pow(link2_length, 2)))/(2*n3*link1_length));

  Serial.println("final angles---");
  for(int i=0; i<=2; i++)
  {
    motorAngles[i] = (asin((num[i])) - atan(k/Z[i]))*180/3.1416;
    Serial.println(motorAngles[i]);   
  }

  /*
  Serial.println("denominator_num3---");
  Serial.println(2*n3*link1_length);

  Serial.println("numerator_num3---");
  Serial.println((pow(link1_length, 2) + pow(n3, 2) - pow(link2_length, 2)));

  Serial.println("num3---");
  Serial.println((num3));
  Serial.println("asin---");
  Serial.println(asin(num3));
  Serial.println("k/z3---");
  Serial.println((k/Z[2]));
  Serial.println("atank/z3---");
  Serial.println(atan(k/Z[2]));
  */
  
}

void LIN_inverseKinematics(float theta_X, float theta_Y)
{
  theta_X *= 3.1416/180;
  theta_Y *= 3.1416/180;

  // Apply eqns and set global motor angles
  float z1_change = (platform_radius/2*(theta_Y)) + (platform_radius*sqrt(3)/2*theta_X);
  float z2_change = (platform_radius/2*(theta_Y)) - (platform_radius*sqrt(3)/2*theta_X);
  float z3_change = -(platform_radius*(theta_Y));

  Z[0] = z_0 + z1_change;
  Z[1] = z_0 + z2_change;
  Z[2] = z_0 + z3_change;

  //clamp z height to avoid weird angles for theta
  for(int i=0;i<=2;i++){
    if (Z[i]>MAX_HEIGHT) Z[i] = MAX_HEIGHT;
    if (Z[i]<MIN_HEIGHT) Z[i] = MIN_HEIGHT;
  }

  Serial.println("final angles---");

  for (int i=0; i<=2; i++){
    motorAngles[i] = -81.2425565391 + 1.18759328456*Z[i];
    Serial.println(motorAngles[i]);
  }
  
}

float PID_Helper(float target_pos, float curr_pos) {
  // Find time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //determine change in time
  prevT = currT; //reset current time

  // Note: Ball Position is in pixels
  int error = target_pos - curr_pos;
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
