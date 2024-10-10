#include <Adafruit_BNO08x.h>
#include <Wire.h>  // Include the Wire library for I2C

#define BNO08X_INT 9       // Define the interrupt pin
#define BNO08X_RESET -1    // Define the reset pin as not used

Adafruit_BNO08x bno08x(BNO08X_RESET);  // Create sensor instance with reset pin
sh2_SensorValue_t sensorValue;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for the serial console to connect

  Serial.println("Adafruit BNO08x test!");

  // Set up the I2C communication on specific pins
  Wire.begin(47, 21);  // SDA on pin 47, SCL on pin 21

  // Try to initialize the sensor using I2C
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);  // Halt the program if the sensor is not detected
    }
  }
  Serial.println("BNO08x Found!");

  // Print sensor ID and version information
  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();  // Set the desired sensor reports

  Serial.println("Reading events");
  delay(100);
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector");
  }
}

void loop() {
  delay(10);

  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset");
    setReports();  // Re-enable reports after a reset
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;  // Skip the rest of the loop if no new data is available
  }

  // Handle the specific sensor data
  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    Serial.print("Game Rotation Vector - r: ");
    Serial.print(sensorValue.un.gameRotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.gameRotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.gameRotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.gameRotationVector.k);
  }
}
