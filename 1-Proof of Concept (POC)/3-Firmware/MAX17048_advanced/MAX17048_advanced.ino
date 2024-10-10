#include "Adafruit_MAX1704X.h"

#define I2C_SDA 47  // Define the SDA pin
#define I2C_SCL 21  // Define the SCL pin

TwoWire MAX17048 = TwoWire(0);  // Create a TwoWire instance called MAX17048
Adafruit_MAX17048 maxlipo;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);    // wait until serial monitor opens

  Serial.println(F("\nAdafruit MAX17048 advanced demo"));

  MAX17048.begin(I2C_SDA, I2C_SCL, 100000);
  if (!maxlipo.begin(&MAX17048)) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(maxlipo.getChipID(), HEX);

  // Quick starting allows an instant 'auto-calibration' of the battery. However, its a bad idea
  // to do this right when the battery is first plugged in or if there's a lot of load on the battery
  // so uncomment only if you're sure you want to 'reset' the chips charge calculator.
  // Serial.println("Quick starting");
  // maxlipo.quickStart();
  
  // The reset voltage is what the chip considers 'battery has been removed and replaced'
  // The default is 3.0 Volts but you can change it here: 
  //maxlipo.setResetVoltage(2.5);
  Serial.print(F("Reset voltage = ")); 
  Serial.print(maxlipo.getResetVoltage());
  Serial.println(" V");

  // Hibernation mode reduces how often the ADC is read, for power reduction. There is an automatic
  // enter/exit mode but you can also customize the activity threshold both as voltage and charge rate

  //maxlipo.setActivityThreshold(0.15);
  Serial.print(F("Activity threshold = ")); 
  Serial.print(maxlipo.getActivityThreshold()); 
  Serial.println(" V change");

  //maxlipo.setHibernationThreshold(5);
  Serial.print(F("Hibernation threshold = "));
  Serial.print(maxlipo.getHibernationThreshold()); 
  Serial.println(" %/hour");

  // You can also 'force' hibernation mode!
  // maxlipo.hibernate();
  // ...or force it to wake up!
  // maxlipo.wake();

  // The alert pin can be used to detect when the voltage of the battery goes below or
  // above a voltage, you can also query the alert in the loop.
  maxlipo.setAlertVoltages(2.0, 4.2);

  float alert_min, alert_max;
  maxlipo.getAlertVoltages(alert_min, alert_max);
  Serial.print("Alert voltages: "); 
  Serial.print(alert_min); Serial.print(" ~ "); 
  Serial.print(alert_max); Serial.println(" V");
}

void loop() {
  Serial.print(F("Batt Voltage: ")); Serial.print(maxlipo.cellVoltage(), 3); Serial.println(" V");
  Serial.print(F("Batt Percent: ")); Serial.print(maxlipo.cellPercent(), 1); Serial.println(" %");
  Serial.print(F("(Dis)Charge rate : ")); Serial.print(maxlipo.chargeRate(), 1); Serial.println(" %/hr");

  // we can check if we're hibernating or not
  if (maxlipo.isHibernating()) {
    Serial.println(F("Hibernating!"));
  }


  if (maxlipo.isActiveAlert()) {
    uint8_t status_flags = maxlipo.getAlertStatus();
    Serial.print(F("ALERT! flags = 0x"));
    Serial.print(status_flags, HEX);
    
    if (status_flags & MAX1704X_ALERTFLAG_SOC_CHANGE) {
      Serial.print(", SOC Change");
      maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_SOC_CHANGE); // clear the alert
    }
    if (status_flags & MAX1704X_ALERTFLAG_SOC_LOW) {
      Serial.print(", SOC Low");
      maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_SOC_LOW); // clear the alert
    }
    if (status_flags & MAX1704X_ALERTFLAG_VOLTAGE_RESET) {
      Serial.print(", Voltage reset");
      maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_RESET); // clear the alert
    }
    if (status_flags & MAX1704X_ALERTFLAG_VOLTAGE_LOW) {
      Serial.print(", Voltage low");
      maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_LOW); // clear the alert
    }
    if (status_flags & MAX1704X_ALERTFLAG_VOLTAGE_HIGH) {
      Serial.print(", Voltage high");
      maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_HIGH); // clear the alert
    }
    if (status_flags & MAX1704X_ALERTFLAG_RESET_INDICATOR) {
      Serial.print(", Reset Indicator");
      maxlipo.clearAlertFlag(MAX1704X_ALERTFLAG_RESET_INDICATOR); // clear the alert
    }
    Serial.println();
  }
  Serial.println();
  Serial.println();

  delay(2000);  // dont query too often!
}