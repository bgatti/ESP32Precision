#include <Wire.h>

#define SDA_PIN 4
#define SCL_PIN 5


const int Motor_A_IN1 = 48;
const int Motor_A_IN2 = 35;
const int Feedback_A = 36;

const int Motor_B_IN1 = 37;
const int Motor_B_IN2 = 38;
const int Feedback_B = 26;

const int Motor_C_IN1 = 41;
const int Motor_C_IN2 = 40;
const int Feedback_C = 39;

volatile int pulseCountA = 0;
volatile int pulseCountB = 0;
volatile int pulseCountC = 0;

// Variables to store high, low, and cumulative values for average computation
int highValueA = 0, lowValueA = 1023, sumValueA = 0, readingsA = 0;
int highValueB = 0, lowValueB = 1023, sumValueB = 0, readingsB = 0;
int highValueC = 0, lowValueC = 1023, sumValueC = 0, readingsC = 0;


void setup() {
    Serial.begin(115200);
    delay(1000);  // Give time for serial monitor

    Serial.print("Analog Value A: ");
    Serial.println(analogRead(Feedback_A));
    Serial.print("Analog Value B: ");
    Serial.println(analogRead(Feedback_B));
    Serial.print("Analog Value C: ");
    Serial.println(analogRead(Feedback_C));


    pinMode(Feedback_A, INPUT_PULLUP);
    pinMode(Feedback_B, INPUT_PULLUP);
    pinMode(Feedback_C, INPUT_PULLUP);

    Serial.print("battery ");
    Serial.println(analogRead(14));

    Serial.print("Analog Value A: ");
    Serial.println(analogRead(Feedback_A));
    Serial.print("Analog Value B: ");
    Serial.println(analogRead(Feedback_B));
    Serial.print("Analog Value C: ");
    Serial.println(analogRead(Feedback_C));




    // Initialize the I2C bus with the given pins
    Wire.begin(SDA_PIN, SCL_PIN);


    pinMode(Motor_A_IN1, OUTPUT);
    pinMode(Motor_A_IN2, OUTPUT);
    pinMode(Feedback_A, INPUT_PULLUP);

    pinMode(Motor_B_IN1, OUTPUT);
    pinMode(Motor_B_IN2, OUTPUT);
    pinMode(Feedback_B, INPUT_PULLUP);

    pinMode(Motor_C_IN1, OUTPUT);
    pinMode(Motor_C_IN2, OUTPUT);
    pinMode(Feedback_C, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(Feedback_A), countPulseA, RISING);
    attachInterrupt(digitalPinToInterrupt(Feedback_B), countPulseB, RISING);
    attachInterrupt(digitalPinToInterrupt(Feedback_C), countPulseC, RISING);
}

void readAnalogValues() {
    int currentValueA = analogRead(Feedback_A);
    int currentValueB = analogRead(Feedback_B);
    int currentValueC = analogRead(Feedback_C);


    Serial.print("Analog Value A: ");
    Serial.println(currentValueA);
    Serial.print("Analog Value B: ");
    Serial.println(currentValueB);
    Serial.print("Analog Value C: ");
    Serial.println(currentValueC);

        

    // Update high and low values for A
    if(currentValueA > highValueA) highValueA = currentValueA;
    if(currentValueA < lowValueA) lowValueA = currentValueA;
    
    // Update high and low values for B
    if(currentValueB > highValueB) highValueB = currentValueB;
    if(currentValueB < lowValueB) lowValueB = currentValueB;

    // Update high and low values for C
    if(currentValueC > highValueC) highValueC = currentValueC;
    if(currentValueC < lowValueC) lowValueC = currentValueC;

    // Update sums for average calculation
    sumValueA += currentValueA;
    sumValueB += currentValueB;
    sumValueC += currentValueC;
    readingsA++;
    readingsB++;
    readingsC++;
}

void loop() {

    pinMode(Feedback_A, INPUT);
    pinMode(Feedback_B, INPUT);
    pinMode(Feedback_C, INPUT);
    

    readAnalogValues();  // Call this function to read and store analog values

    pinMode(Feedback_A, INPUT_PULLUP);
    pinMode(Feedback_B, INPUT_PULLUP);
    pinMode(Feedback_C, INPUT_PULLUP);

  
    Serial.println("Testing Standby Mode");
    setMotorMode(LOW, LOW); // Standby
    delay(10);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Forward Mode");
    setMotorMode(LOW, HIGH); // Forward
    delay(10);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Brake Mode");
    setMotorMode(HIGH, HIGH); // Brake
    delay(500);
    reportPulses();
    resetPulseCounts();

    Serial.println("Quiet Section (Brake Mode)");
    setMotorMode(HIGH, HIGH); // Brake
    delay(500);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Reverse Mode");
    setMotorMode(HIGH, LOW); // Reverse
    delay(10);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Brake Mode");
    setMotorMode(HIGH, HIGH); // Brake
    delay(100);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Forward Mode");
    setMotorMode(LOW, HIGH); // Forward
    delay(10);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Brake Mode");
    setMotorMode(HIGH, HIGH); // Brake
    delay(100);
    reportPulses();
    resetPulseCounts();

    // Report analog readings
    Serial.print("Motor A Analog High: ");
    Serial.println(highValueA);
    Serial.print("Motor A Analog Low: ");
    Serial.println(lowValueA);
    Serial.print("Motor A Analog Avg: ");
    Serial.println(sumValueA / readingsA);

    Serial.print("Motor B Analog High: ");
    Serial.println(highValueB);
    Serial.print("Motor B Analog Low: ");
    Serial.println(lowValueB);
    Serial.print("Motor B Analog Avg: ");
    Serial.println(sumValueB / readingsB);

    Serial.print("Motor C Analog High: ");
    Serial.println(highValueC);
    Serial.print("Motor C Analog Low: ");
    Serial.println(lowValueC);
    Serial.print("Motor C Analog Avg: ");
    Serial.println(sumValueC / readingsC);    


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




    delay(1000); // Optional delay between cycles
}

void setMotorMode(bool in1, bool in2) {
    digitalWrite(Motor_A_IN1, in1);
    digitalWrite(Motor_A_IN2, in2);
    digitalWrite(Motor_B_IN1, in1);
    digitalWrite(Motor_B_IN2, in2);
    digitalWrite(Motor_C_IN1, in1);
    digitalWrite(Motor_C_IN2, in2);
}

void countPulseA() {
    pulseCountA++;
}

void countPulseB() {
    pulseCountB++;
}

void countPulseC() {
    pulseCountC++;
}

void reportPulses() {
    Serial.print("Motor A Pulses: ");
    Serial.println(pulseCountA);
    Serial.print("Motor B Pulses: ");
    Serial.println(pulseCountB);
    Serial.print("Motor C Pulses: ");
    Serial.println(pulseCountC);

    Serial.print("Motor A feedback State: ");
    Serial.println(digitalRead(Feedback_A));
    Serial.print("Motor B feedback State: ");
    Serial.println(digitalRead(Feedback_B));
    Serial.print("Motor C feedback State: ");
    Serial.println(digitalRead(Feedback_C));

    
}

void resetPulseCounts() {
    pulseCountA = 0;
    pulseCountB = 0;
    pulseCountC = 0;
}
