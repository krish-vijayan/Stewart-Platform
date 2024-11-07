
const int dirPinStepper1 = 2; //direction Pin (DIR+)
const int stepPinStepper1 = 3; //pulse Pin (PUL+)
const int enPinStepper1 = 4; //enable Pin (ENA+)

const int dirPinStepper2 = 6; //direction Pin (DIR+))
const int stepPinStepper2 = 7; //pulse Pin (PUL+)
const int enPinStepper2 = 5; //enable Pin (ENA+)

const int dirPinStepper3 = 9; //direction Pin (DIR+))
const int stepPinStepper3 = 8; //pulse Pin (PUL+)
const int enPinStepper3 = 10; //enable Pin (ENA+)

// Motor Angle Definitions
struct motorAngles { 
    float motor1;
    float motor2;
    float motor3;
};

// Definitions
// Motor Driver 1
// Pinouts (D3 (PUL+), D4 (ENA+), D2 (DIR+))

// Motor Driver 2
// Pinouts (D5 (ENA+), D6 (DIR+), D7 (PUL+))

// Motor Driver 3
// Pinouts (D8 (PUL+), D9 (DIR+), D10 (ENA+))