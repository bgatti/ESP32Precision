#include <Wire.h>
#include "LIS2DW12Sensor.h"

#define I2C_SDA 4  // You might need to adjust these pins
#define I2C_SCL 5  // You might need to adjust these pins


TwoWire dev_i2c(0);
LIS2DW12Sensor Accelero(&dev_i2c);

void reportPinState(uint8_t pin) {
    pinMode(pin, INPUT);
    delay(10);  // Allow for pins to settle
    if (digitalRead(pin) == HIGH) {
        Serial.print("Pin ");
        Serial.print(pin);
        Serial.println(" is HIGH in Hi-Z mode.");
    } else {
        Serial.print("Pin ");
        Serial.print(pin);
        Serial.println(" is LOW in Hi-Z mode.");
    }

    pinMode(pin, INPUT_PULLUP);
    delay(10);  // Allow for pins to settle
    if (digitalRead(pin) == HIGH) {
        Serial.print("Pin ");
        Serial.print(pin);
        Serial.println(" is HIGH with internal pull-up.");
    } else {
        Serial.print("Pin ");
        Serial.print(pin);
        Serial.println(" is LOW even with internal pull-up. External pull-down might be present or the pin is shorted to ground.");
    }
}



void setup() {
  delay(1000);  //usb will reset uart on some platforms
  Serial.begin(115200);


  Serial.println("Reporting pin states...");
  reportPinState(I2C_SDA);
  reportPinState(I2C_SCL);
  

  dev_i2c.begin(I2C_SDA, I2C_SCL);
  dev_i2c.setClock(400000);  // Set speed to 400kHz
  
  delay(1000);

  

  while (Accelero.begin() != LIS2DW12_STATUS_OK) {
    Serial.println("LIS2DW12 initialization failed! Retrying...");
    delay(1000);  // Delay for 1 second before retrying
  }


  if (Accelero.Enable_X() != LIS2DW12_STATUS_OK) {
    Serial.println("Failed to enable X axis!");
  } else {
    Serial.println("X axis enabled successfully!");
  }



  // I2C Scanner to identify connected devices and their addresses
  Serial.println("Scanning I2C devices...");
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++ ) {
    dev_i2c.beginTransmission(address);
    error = dev_i2c.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.println("I2C scan completed.");
  }

  // Initialize Accelerometer
  if (Accelero.begin() == LIS2DW12_STATUS_OK) {
    Serial.println("LIS2DW12 initialization success!");
  } else {
    Serial.println("LIS2DW12 initialization failed!");
    while (1);  // Halt execution if initialization fails
  }
  
  // Read and display the WHO_AM_I register
  uint8_t whoAmI; 
  Accelero.ReadID(&whoAmI);
  Serial.print("WHO_AM_I Register: 0x");
  Serial.println(whoAmI, HEX);


    // Get the ODR to verify the setting
    float readODR;
    if (Accelero.Get_X_ODR(&readODR) == LIS2DW12_STATUS_OK) {
        Serial.print("Read ODR: ");
        Serial.println(readODR);
    } else {
        Serial.println("Failed to read ODR.");
    }

    readODR = readODR/2;    
    if (Accelero.Set_X_ODR(readODR) == LIS2DW12_STATUS_OK) {
        Serial.print("Read ODR: ");
        Serial.println("ODR set successfully.");
    } else {
        Serial.println("Failed to set ODR.");
    }

    if (Accelero.Get_X_ODR(&readODR) == LIS2DW12_STATUS_OK) {
        Serial.print("Read ODR: ");
        Serial.println(readODR);
    } else {
        Serial.println("Failed to read ODR.");
    }

    
      
}

void loop() {

  int32_t accelerometer[3];

  
  if (Accelero.Get_X_Axes(accelerometer) != LIS2DW12_STATUS_OK) {
    Serial.println("Failed to get X axes data!");
  } else {
    Serial.print("X: ");
    Serial.print(accelerometer[0]);
    Serial.print(" Y: ");
    Serial.print(accelerometer[1]);
    Serial.print(" Z: ");
    Serial.println(accelerometer[2]);
  }

  delay(200);



  
}
