#include <Wire.h>

const int SDA_Pin = 20;
const int SCL_Pin = 21;

int x = 0;
int y = 0;
int byteCounter = 0;  // Track which byte we're reading
int x_high = 0, x_low = 0, y_high = 0, y_low = 0;


void setup() {
  Serial.begin(9600);

  Wire.begin(0x8);

  // Attach a function to trigger when data is received
  Wire.onReceive(receiveEvent);

  // Start serial communication for debugging

  // Ensure pull-up resistors on the I2C lines are disabled
  pinMode(SDA_Pin, INPUT);
  pinMode(SCL_Pin, INPUT);
}

void receiveEvent(int bytes) {
    while (Wire.available()) {
        if (byteCounter == 0) {
            x_high = Wire.read();  // First byte is the high byte of x
        } else if (byteCounter == 1) {
            x_low = Wire.read();   // Second byte is the low byte of x
            x = (x_high << 8) | x_low;  // Combine high and low bytes into x
        } else if (byteCounter == 2) {
            y_high = Wire.read();  // Third byte is the high byte of y
        } else if (byteCounter == 3) {
            y_low = Wire.read();   // Fourth byte is the low byte of y
            y = (y_high << 8) | y_low;  // Combine high and low bytes into y

            // Debugging output to verify the received values
            Serial.print("Received X: ");
            Serial.print(x);
            Serial.print(", Y: ");
            Serial.println(y);

            // Reset the counter after receiving all 4 bytes
            byteCounter = -1;
        }
        byteCounter++;
    }
}



void loop() {
  

  delay(500);  // Delay to avoid spamming the output
}
