#include <Wire.h>

#define SDA_PIN 4
#define SCL_PIN 5

void setup() {
  Serial.begin(115200);
  
  // Initialize the I2C bus with the given pins
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000);  // Give time for serial monitor
}

void loop() {
  // No repeated action required for this scanner
  delay(1000);

  Serial.println("I2C device scanner. Scanning ...");
  byte count = 0;

  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      if (i < 16) {
        Serial.print("0");
      }
      Serial.print(i, HEX);
      Serial.println(" !");

      count++;
      delay(1);  // Maybe give time to the device
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.println("done.");  
  }


  
}
