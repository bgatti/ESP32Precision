class ServoMotor {
private:
    int pinIN1, pinIN2, feedbackPin;
    int desiredPosition;
    enum class Mode { FORWARD, REVERSE, BRAKE, IDLE } currentMode = Mode::IDLE;
    void (*interruptHandler)();
    int power;

public:
    int currentPosition;
    int lastDirection;

    ServoMotor(int IN1, int IN2, int feedback, void (*handler)()) : pinIN1(IN1), pinIN2(IN2), feedbackPin(feedback), currentPosition(0), desiredPosition(0), lastDirection(0), interruptHandler(handler) {
        pinMode(pinIN1, OUTPUT);
        pinMode(pinIN2, OUTPUT);
        pinMode(feedbackPin, INPUT_PULLUP);
//        attachInterrupt(digitalPinToInterrupt(feedbackPin), interruptHandler, RISING);
    }

    void setPosition(int position) {
        desiredPosition = position;
    }

    void move(int count) {
      
        currentPosition += (count * lastDirection);

        if(currentMode == Mode::BRAKE && count > 0) // continue braking until stopped
          return;        
        
        int p = 30;
        

        if (count > 5) // maintain speed below x
          power /= 2;        
        else
          power = (abs(currentPosition - desiredPosition) * p);

        power = min(power,255);

                      
        if (currentPosition == desiredPosition) {
            setMode(Mode::IDLE);
            return;
        }
        
        if (currentPosition < desiredPosition) {
            if (currentMode == Mode::REVERSE) {
                setMode(Mode::BRAKE);
            } else {
                setMode(Mode::FORWARD);
            }
        } else if (currentPosition > desiredPosition) {
            if (currentMode == Mode::FORWARD) {
                setMode(Mode::BRAKE);
            } else {
                setMode(Mode::REVERSE);
            }
        } else {
            setMode(Mode::BRAKE);
        }
        
    }

    void rest() {
        if (currentMode != Mode::BRAKE)
          setMode(Mode::IDLE);
    }

   void setMode(Mode mode) {
      switch (mode) {
          case Mode::FORWARD:
              analogWrite(pinIN1, 0);         // 0% duty cycle
              analogWrite(pinIN2, power);    // N% duty cycle
              lastDirection = 1;
              break;
          case Mode::REVERSE:
              analogWrite(pinIN1, power);    // N% duty cycle
              analogWrite(pinIN2, 0);        // 0% duty cycle
              lastDirection = -1;
              break;
          case Mode::BRAKE:
              analogWrite(pinIN1, power);    // N% duty cycle
              analogWrite(pinIN2, power);    // N% duty cycle
              break;
          case Mode::IDLE:
              analogWrite(pinIN1, 0);        // 0% duty cycle
              analogWrite(pinIN2, 0);        // 0% duty cycle
              break;
      }
      currentMode = mode;
  }


    void updatePositionBasedOnDirection() {
   //     currentPosition += lastDirection;
    }
};

// Forward declare the global motorA instance
extern ServoMotor motorA;
int dummy;
int countA;

// Global interrupt handler for Motor A
void handleInterruptA() {
//    motorA.updatePositionBasedOnDirection();
    countA++;
  //  dummy++;
}

// Global instance for the motor
ServoMotor motorA(48, 35, 36, handleInterruptA);


void setup() {
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(36), handleInterruptA, RISING);

    
}

unsigned long mil;
unsigned long restmil;
unsigned long last;
unsigned long lastSerial;

void loop() {
    int position = 50 * sin(millis() * PI / 500);

    if ( millis() > mil)
    {
      motorA.setPosition(position);
      motorA.move(countA);
      
      countA = 0;
      restmil = millis() + 3;
      mil = millis() + 500;
    }
    
    if ( millis() > restmil)
    {
      motorA.rest();
      mil = millis() + 10;    
      restmil = millis() + 500;
    }

    if( false)// millis() > lastSerial && countA > 0)
    {

      Serial.print(position);    
      Serial.print("  ");     
      Serial.print(countA);    
      Serial.print("  ");     
      Serial.print(motorA.lastDirection);    
      Serial.print("  ");     
      Serial.println(motorA.currentPosition);

      
      lastSerial = millis()+ 100;
    }

    if (dummy > 100)
      {
      Serial.println(millis()-last);    
      dummy = 0;
      last = millis();
      }
      

//    
    

}
