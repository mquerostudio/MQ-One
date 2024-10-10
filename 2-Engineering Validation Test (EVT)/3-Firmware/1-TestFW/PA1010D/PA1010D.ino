#include <Adafruit_GPS.h>
#include <Wire.h>  // Include the Wire library to handle I2C

// Connect to the GPS on a specified I2C port
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

void setup()
{
  Serial.begin(115200);
  Serial.println("Adafruit I2C GPS library basic test!");

  // Initialize the Wire library with SDA on pin 11 and SCL on pin 10
  Wire.begin(9, 10);

  // Initialize the GPS module at the I2C address 0x10
  GPS.begin(0x10);

  // Set the desired NMEA sentences and update rate
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Request firmware version
  GPS.println(PMTK_Q_RELEASE);
}

void loop()
{
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);

  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) Serial.print('0');
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) Serial.print('0');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) Serial.print('0');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}
