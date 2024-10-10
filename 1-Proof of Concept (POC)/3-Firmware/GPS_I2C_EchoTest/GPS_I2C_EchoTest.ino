#include <Wire.h>            // Include the I2C library (this is a standard library in Arduino)
#include <Adafruit_GPS.h>    // Include the Adafruit GPS library

// Connect to the GPS on the specified I2C port
Adafruit_GPS GPS(&Wire);

void setup() {
  // wait for hardware serial to appear
  while (!Serial) ;

  // make this baud rate fast enough so we aren't waiting on it
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic I2C test!");

  Wire.begin(47, 21); 
  GPS.begin(0x10);  // The I2C address to use is 0x10
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    GPS.write(c);
  }
  if (GPS.available()) {
    char c = GPS.read();
    Serial.write(c);
  }
}
