// Includes
#include <LIS2DW12Sensor.h>



#define I2C_SDA 4  // You might need to adjust these pins
#define I2C_SCL 5  // You might need to adjust these pins


#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

// Components
LIS2DW12Sensor Acc2(&DEV_I2C);

void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);
  
  // Initialize I2C bus.
  DEV_I2C.begin(I2C_SDA, I2C_SCL);
  
  Acc2.begin();
  Acc2.Enable_X();
}

void loop() {
  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  //Read accelerometer
  int32_t accelerometer2[3];
  Acc2.Get_X_Axes(accelerometer2);

  // Output data.
  SerialPort.print(" | Acc2[mg]: ");
  SerialPort.print(accelerometer2[0]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer2[1]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer2[2]);
  SerialPort.println(" |");
}
