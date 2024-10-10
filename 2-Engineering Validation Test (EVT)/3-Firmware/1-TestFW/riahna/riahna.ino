#include <SPI.h>
#include "Adafruit_seesaw.h"
#include <TFT_eSPI.h>  // Hardware-specific library
#include "Audio.h"
#include "SPI.h"
#include "SD.h"
#include "FS.h"
#include "Arduino.h"

#define SD_CS 1
#define SD_MISO 44
#define SD_MOSI 2
#define SD_SCLK 43

#define I2S_BCLK 12
#define I2S_LRC 14
#define I2S_DOUT 11

#define SS_NEOPIX 20
#define SEESAW_ADDR 0x49

Adafruit_seesaw seesaw;
TFT_eSPI tft = TFT_eSPI();  // Invoke custom library
Audio audio;

void setup(void) {
  Serial.begin(115200);
  Wire.begin(47, 21);         // SDA on pin 47, SCL on pin 21
  while (!Serial) delay(10);  // wait until serial port is opened

  delay(100);
  Serial.println(F("Seesaw Init"));
  if (!seesaw.begin()) {
    Serial.println(F("seesaw not found!"));
    while (1) delay(10);
  }
  delay(100);

  seesaw.turnOn();
  delay(100);

  tft.init();
  tft.setRotation(4);

  tft.fillScreen(TFT_BLACK);
  delay(1000);
  tft.setCursor(50, 140, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.println("Don't Stop The Music");
  tft.setTextSize(2);
  tft.setCursor(60, 200, 2);
  tft.println("papapaula");

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);
  SPI.setFrequency(1000000);
  Serial.begin(115200);
  SD.begin(SD_CS);
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21);  // 0...21
  audio.connecttoFS(SD, "test.mp3");
  seesaw.turnOff();

}

void loop() {
  audio.loop();
  // seesaw.turnOff();
}