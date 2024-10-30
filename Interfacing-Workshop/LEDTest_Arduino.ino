#include <Wire.h>

const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int ledPin = 13; // *** Change LED Pin here

int LED_Byte = 0;
 
void setup() {
  // Arduino joins I2C bus as slave with address 8
  Wire.begin(0x8);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);
  
  // Setup pin 13 as output and turn LED off at the beginning
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Turn off 20k-50k ohm built-in pull up resistors at pins specified
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}
 
// Function that executes whenever data is received from master device, the Pi 5
void receiveEvent(int howMany) {
  LED_Byte = Wire.read(); // receive byte as an integer
  digitalWrite(ledPin, LED_Byte); // turn on/off LED based on byte information
  Serial.println(LED_Byte);
  
}

void loop() {
  delay(100); // Keep waiting for data
}

