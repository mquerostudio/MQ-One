#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#define SS_SWITCH 24
#define SS_NEOPIX 6
#define LED_COUNT 24 // Number of LEDs you have
#define SEESAW_ADDR 0x49

Adafruit_seesaw ss;
seesaw_NeoPixel sspixel = seesaw_NeoPixel(LED_COUNT, SS_NEOPIX, NEO_GRB + NEO_KHZ800);

int32_t encoder_position;
bool loadingMode = false; // To keep track of whether we're in loading mode
unsigned long lastDebounceTime = 0; // Last time the switch was toggled
unsigned long debounceDelay = 500; // The debounce time; increase if the output flickers
int loadingPosition = 0; // Position for loading animation

void setup() {
  Wire.begin(47, 21);
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Looking for seesaw!");

  if (!ss.begin(SEESAW_ADDR) || !sspixel.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while (1) delay(10);
  }
  Serial.println("seesaw started");

  uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
  if (version != 1402) {
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
    while (1) delay(10);
  }

  sspixel.setBrightness(20); // Set brightness to a reasonable level
  sspixel.show(); // Initialize all pixels to 'off'

  ss.pinMode(SS_SWITCH, INPUT_PULLUP); // Setup the switch pin mode
  encoder_position = ss.getEncoderPosition(); // Get the initial encoder position

  Serial.println("Turning on interrupts");
  delay(10);
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt(); // Enable encoder interrupts
}

void addShadeEffect(int pos, uint32_t color) {
  // Set the main color for the current position
  sspixel.setPixelColor(pos % LED_COUNT, color);
  
  // Calculate the dimmed colors for the shading effect
  uint8_t r = (uint8_t)(color >> 16);   // Extract the red component
  uint8_t g = (uint8_t)(color >> 8);    // Extract the green component
  uint8_t b = (uint8_t)color;           // Extract the blue component
   
  // Calculate shades for extended fade effect
  uint32_t dimColor1 = sspixel.Color(r / 2, g / 2, b / 2); // 50% brightness
  uint32_t dimColor2 = sspixel.Color(r / 4, g / 4, b / 4); // 25% brightness
  uint32_t dimColor3 = sspixel.Color(r / 8, g / 8, b / 8); // 12.5% brightness

  // Apply the dimmed colors to surrounding LEDs for a longer shade effect
  // Adjust the range and effect as necessary
  int prev1 = (pos - 1 + LED_COUNT) % LED_COUNT;
  int prev2 = (pos - 2 + LED_COUNT) % LED_COUNT;
  int prev3 = (pos - 3 + LED_COUNT) % LED_COUNT;
  int next1 = (pos + 1) % LED_COUNT;
  int next2 = (pos + 2) % LED_COUNT;
  int next3 = (pos + 3) % LED_COUNT;

  sspixel.setPixelColor(prev1, dimColor1);
  sspixel.setPixelColor(prev2, dimColor2);
  sspixel.setPixelColor(prev3, dimColor3);
  sspixel.setPixelColor(next1, dimColor1);
  sspixel.setPixelColor(next2, dimColor2);
  sspixel.setPixelColor(next3, dimColor3);
}



void loop() {
  // Read the state of the switch into a local variable:
  int reading = ss.digitalRead(SS_SWITCH);

  // Check if the button is pressed (taking into account the debounce delay)
  if (reading == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    // reset the debouncing timer
    lastDebounceTime = millis();

    // toggle the loading mode
    loadingMode = !loadingMode;
    loadingPosition = 0; // Reset loading position
    Serial.println("Button pressed - toggle loading mode");
  }

  if (loadingMode) {
    // In loading mode, display a loading animation with shade effect
    for (int i = 0; i < LED_COUNT; i++) {
      sspixel.setPixelColor(i, 0); // Turn off all LEDs initially
    }

    // Add shade effect around the loading position
    addShadeEffect(loadingPosition, sspixel.Color(0, 0, 255)); // Blue color for loading effect

    sspixel.show();

    // Move to the next position
    loadingPosition = (loadingPosition + 1) % LED_COUNT;
    delay(100); // Slow down the loading animation
  } else {
    // Normal mode - handle encoder and LED logic as before
    int32_t new_position = ss.getEncoderPosition();
    if (encoder_position != new_position) {
      Serial.println(new_position);

      for (int i = 0; i < LED_COUNT; i++) {
        sspixel.setPixelColor(i, 0); // Turn off all LEDs
      }

      if (new_position >= 0) {
        int upperBound = min(new_position, LED_COUNT - 1);
        for (int i = 0; i <= upperBound; i++) {
          sspixel.setPixelColor(i, sspixel.Color(0, 255, 0)); // Green for positive
        }
      } else {
        int startLED = LED_COUNT + new_position;
        for (int i = LED_COUNT - 1; i >= startLED; i--) {
          sspixel.setPixelColor(i, sspixel.Color(255, 0, 0)); // Red for negative
        }
      }

      sspixel.show();
      encoder_position = new_position;
    }
  }

  // Debounce delay to prevent flickering in the LED state
  delay(10);
}
