// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>

#define I2C_SDA 47  // Define the SDA pin
#define I2C_SCL 21  // Define the SCL pin

#define RESET_PIN -1

TwoWire BNO085 = TwoWire(0);  // Create a TwoWire instance called BNO085
Adafruit_BNO08x  bno08x(RESET_PIN);  // Create an Adafruit_BNO08x instance called bno08x
sh2_SensorValue_t sensorValue;  // Structure to hold sensor data

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // Will pause until serial console opens

  Serial.println("Adafruit BNO08x test!");

  BNO085.begin(I2C_SDA, I2C_SCL, 100000);  // Initialize the BNO085 I2C communication with custom SDA and SCL pins
  // Try to initialize the BNO08x sensor
  if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &BNO085)) {  // Specify the I2C address and the I2C instance
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }  // Infinite loop if sensor is not found
  }
  Serial.println("BNO08x Found!");

  // Print the product IDs and software versions of the sensor
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

  setReports();  // Set desired sensor reports

  Serial.println("Reading events");
  delay(100);
}

// Define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}

void loop() {
  delay(10);

  // Check if sensor was reset and re-enable reports if necessary
  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset ");
    setReports();
  }
  
  // Retrieve sensor events; skip this loop iteration if there's none
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  
  // Process sensor events
  switch (sensorValue.sensorId) {
    
    case SH2_GAME_ROTATION_VECTOR:
      Serial.print("Game Rotation Vector - r: ");
      Serial.print(sensorValue.un.gameRotationVector.real);
      Serial.print(" i: ");
      Serial.print(sensorValue.un.gameRotationVector.i);
      Serial.print(" j: ");
      Serial.print(sensorValue.un.gameRotationVector.j);
      Serial.print(" k: ");
      Serial.println(sensorValue.un.gameRotationVector.k);
      break;
  }

}
