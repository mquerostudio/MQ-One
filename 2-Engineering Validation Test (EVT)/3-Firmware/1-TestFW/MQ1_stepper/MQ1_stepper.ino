#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#include <SPI.h>
#include <TFT_eSPI.h>  // Hardware-specific library

#include <AccelStepper.h>

#include <PNGdec.h>
#include "interface.h"  // Image is stored here in an 8-bit array

#define TFT_CS 40
#define TFT_RST 41  // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC 42
#define TFT_MOSI 39  // Data out
#define TFT_SCLK 38  // Clock out

#define NEOPIXEL_PIN 20
#define LED_COUNT 24  // Number of LEDs you have
#define PWR_PIN 7
#define ENCODER_SW_PIN 48

#define MAX_IMAGE_WIDTH 240  // Adjust for your images
#define DELAY_IMAGE 20

#define DIR_PIN 15
#define STEP_PIN 16
#define MOTOR_STEPS 200

Adafruit_seesaw seesaw;
PNG png;                                      // PNG decoder instance
AccelStepper stepper1(1, STEP_PIN, DIR_PIN);  // (Type of driver: with 2 pins, STEP, DIR)
seesaw_NeoPixel strip = seesaw_NeoPixel(24, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

int16_t xpos = 0;
int16_t ypos = 0;

int32_t encoder_position;

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

float speed = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(47, 21);         // SDA on pin 47, SCL on pin 21
  while (!Serial) delay(10);  // wait until serial port is opened

  delay(100);
  Serial.println(F("Seesaw Init"));
  if (!seesaw.begin()) {
    Serial.println(F("seesaw not found!"));
    while (1) delay(10);
  }

  Serial.println(F("Neopixel Init"));
  if (!strip.begin()) {
    Serial.println(F("seesaw pixels not found!"));
    while (1) delay(10);
  }
  Serial.println(F("seesaw started OK!"));

  delay(100);

  seesaw.turnOn();
  pinMode(ENCODER_SW_PIN, INPUT);

  // get starting position
  encoder_position = seesaw.getEncoderPosition();
  seesaw.enableEncoderInterrupt();

  stepper1.setMaxSpeed(1000*32);
  stepper1.setSpeed(0);

  delay(2000);

  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
  xTaskCreatePinnedToCore(Task3code, "Task3", 10000, NULL, 1, &Task3, 1);
  xTaskCreatePinnedToCore(Task4code, "Task4", 10000, NULL, 2, &Task4, 1);
}

void Task1code(void *pvParameters) {
  tft.begin();

  float oldspeed = 0;
  int16_t rc;
  int counter = 1;

  rc = png.openFLASH((uint8_t *)base, sizeof(base), pngDraw);
  if (rc == PNG_SUCCESS) {
    tft.startWrite();
    rc = png.decode(NULL, 0);
    tft.endWrite();
    // png.close(); // not needed for memory->memory decode
  }


  for (;;) {
    while (speed != 0) {
      switch (counter) {

        case 1:
          // Print Top Arrow
          rc = png.openFLASH((uint8_t *)stepperArrowTop, sizeof(stepperArrowTop), pngDraw);
          if (rc == PNG_SUCCESS) {
            tft.startWrite();
            rc = png.decode(NULL, 0);
            tft.endWrite();
            tft.endWrite();
            // png.close(); // Required for files, not needed for FLASH arrays
          }
          counter = 2;
          delay(DELAY_IMAGE);
          break;

        case 2:
          // Print Right Arrow
          rc = png.openFLASH((uint8_t *)stepperArrowRight, sizeof(stepperArrowRight), pngDraw);
          if (rc == PNG_SUCCESS) {
            tft.startWrite();
            rc = png.decode(NULL, 0);
            tft.endWrite();
            tft.endWrite();
            // png.close(); // Required for files, not needed for FLASH arrays
          }
          counter = 3;
          delay(DELAY_IMAGE);
          break;

        case 3:
          // Print Bot Arrow
          rc = png.openFLASH((uint8_t *)stepperArrowBot, sizeof(stepperArrowBot), pngDraw);
          if (rc == PNG_SUCCESS) {
            tft.startWrite();
            rc = png.decode(NULL, 0);
            tft.endWrite();
            tft.endWrite();
            // png.close(); // Required for files, not needed for FLASH arrays
          }
          counter = 4;
          delay(DELAY_IMAGE);
          break;

        case 4:
          // Print Left Arrow
          rc = png.openFLASH((uint8_t *)stepperArrowLeft, sizeof(stepperArrowLeft), pngDraw);
          if (rc == PNG_SUCCESS) {
            tft.startWrite();
            rc = png.decode(NULL, 0);
            tft.endWrite();
            tft.endWrite();
            // png.close(); // Required for files, not needed for FLASH arrays
          }
          counter = 1;
          delay(DELAY_IMAGE);
          break;
      }

      if (speed != oldspeed) {
        // Print BG Arrow
        rc = png.openFLASH((uint8_t *)SpeedBG, sizeof(SpeedBG), pngDraw);
        if (rc == PNG_SUCCESS) {
          tft.startWrite();
          rc = png.decode(NULL, 0);
          tft.endWrite();
          tft.endWrite();
          // png.close(); // Required for files, not needed for FLASH arrays
        }
        tft.setCursor(80, 100, 2);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.println(speed);
        oldspeed = speed;

        stepper1.setSpeed(speed*32);
      }
    }

    if (speed == 0) {

      if (speed != oldspeed) {
        // Print BG Arrow
        rc = png.openFLASH((uint8_t *)SpeedBG, sizeof(SpeedBG), pngDraw);
        if (rc == PNG_SUCCESS) {
          tft.startWrite();
          rc = png.decode(NULL, 0);
          tft.endWrite();
          tft.endWrite();
          // png.close(); // Required for files, not needed for FLASH arrays
        }
        tft.setCursor(80, 100, 2);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.println(speed);
        oldspeed = speed;
      }

      rc = png.openFLASH((uint8_t *)stepper, sizeof(stepper), pngDraw);
      if (rc == PNG_SUCCESS) {
        tft.startWrite();
        rc = png.decode(NULL, 0);
        tft.endWrite();
        tft.endWrite();
        // png.close(); // Required for files, not needed for FLASH arrays
      }

      counter = 1;
      delay(10);
    }
  }
}

void Task2code(void *pvParameters) {
  strip.setBrightness(100);
  strip.show();

  for (;;) {
    int32_t new_position = seesaw.getEncoderPosition();

    if (encoder_position != new_position) {
      Serial.println(new_position);  // display new position

      for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, 0);  // Turn off all LEDs
      }

      if (new_position >= 0) {
        int upperBound = min(new_position, static_cast<int32_t>(LED_COUNT - 1));
        for (int i = 0; i <= upperBound; i++) {
          strip.setPixelColor(i, strip.Color(0, 255, 0));  // Green for positive
        }
      } else {
        int startLED = LED_COUNT + new_position;
        for (int i = LED_COUNT - 1; i >= startLED; i--) {
          strip.setPixelColor(i, strip.Color(255, 0, 0));  // Red for negative
        }
      }

      strip.show();
      encoder_position = new_position;  // and save for next round
    }

    speed = encoder_position * 10;
    Serial.println(speed);
    delay(100);
  }
}

void Task3code(void *pvParameters) {
  for (;;) {
    stepper1.runSpeed();
  }
}
void Task4code(void *pvParameters) {
  for (;;) {
    if (!digitalRead(ENCODER_SW_PIN)) {
      seesaw.turnOff();
    }
    delay(500);
  }
}

void loop() {}
