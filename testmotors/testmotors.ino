const int Motor_A_IN1 = 48;
const int Motor_A_IN2 = 35;
const int Feedback_A = 36;

const int Motor_B_IN1 = 37;
const int Motor_B_IN2 = 38;
const int Feedback_B = 39;

const int Motor_C_IN1 = 41;
const int Motor_C_IN2 = 40;
const int Feedback_C = 26;

volatile int pulseCountA = 0;
volatile int pulseCountB = 0;
volatile int pulseCountC = 0;

void setup() {
    Serial.begin(115200);

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

void loop() {
    Serial.println("Testing Standby Mode");
    setMotorMode(LOW, LOW); // Standby
    delay(10);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Brake Mode");
    setMotorMode(HIGH, HIGH); // Brake
    delay(10);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Forward Mode");
    setMotorMode(LOW, HIGH); // Forward
    delay(50);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Brake Mode");
    setMotorMode(HIGH, HIGH); // Brake
    delay(10);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Reverse Mode");
    setMotorMode(HIGH, LOW); // Reverse
    delay(50);
    reportPulses();
    resetPulseCounts();

    Serial.println("Testing Brake Mode");
    setMotorMode(HIGH, HIGH); // Brake
    delay(10);
    reportPulses();
    resetPulseCounts();

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
