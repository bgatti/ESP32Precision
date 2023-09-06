

const int Motor_A_IN1 = 48;
const int Motor_A_IN2 = 35;
const int Feedback_A = 36;

const int Motor_B_IN1 = 37;
const int Motor_B_IN2 = 38;
const int Feedback_B = 26;

const int Motor_C_IN1 = 41;
const int Motor_C_IN2 = 40;
const int Feedback_C = 39;


void setup() {
    Serial.begin(115200);
    delay(1000);  // Give time for serial monitor

 

}

void loop() {
  // put your main code here, to run repeatedly:

    pinMode(Feedback_A, INPUT_PULLUP);
    pinMode(Feedback_B, INPUT_PULLUP);
    pinMode(Feedback_C, INPUT_PULLUP);

    delay(200);

    Serial.print("battery ");
    Serial.println(analogRead(14));
   
   Serial.print("Analog Value A: ");
    Serial.println(analogRead(Feedback_A));
    Serial.print("Analog Value B: ");
    Serial.println(analogRead(Feedback_B));
    Serial.print("Analog Value C: ");
    Serial.println(analogRead(Feedback_C));


    pinMode(Feedback_A, INPUT);
    pinMode(Feedback_B, INPUT);
    pinMode(Feedback_C, INPUT);

   Serial.print("Analog Value A: ");
    Serial.println(analogRead(Feedback_A));
    Serial.print("Analog Value B: ");
    Serial.println(analogRead(Feedback_B));
    Serial.print("Analog Value C: ");
    Serial.println(analogRead(Feedback_C));
    delay(200);

    

    
}
