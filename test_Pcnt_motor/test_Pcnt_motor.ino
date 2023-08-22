#include "driver/gpio.h"
#include "driver/pcnt.h"

#define PCNT_INPUT_SIG_IO  36  // Pulse Input GPIO



#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      32000
#define PCNT_L_LIM_VAL     -32000
#define PCNT_THRESH1_VAL    5
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   36  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  4   // Control GPIO HIGH=count up, LOW=count down
#define FILTER_VALUE        1666  // The computed filter value for 3000 Hz



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
        
        int p = 50;
        

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
 //   countA++;
  //  dummy++;
}

// Global instance for the motor
ServoMotor motorA(48, 35, 36, handleInterruptA);

   // Set up pulse counter module
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        .pos_mode = PCNT_COUNT_INC,      // Increment the counter for an incoming pulse
        .neg_mode = PCNT_COUNT_DIS,      // Do not change counter on the negative edge
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
        .unit = PCNT_TEST_UNIT,
        .channel = PCNT_CHANNEL_0
    };

void configurePullUp() {
    gpio_config_t io_conf;
    // Configure the pin as an input
    io_conf.mode = GPIO_MODE_INPUT;
    // Enable pull-up
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    // Disable pull-down
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // Set the GPIO pin to configure
    io_conf.pin_bit_mask = (1ULL << PCNT_INPUT_SIG_IO);
    // Apply the configuration
    gpio_config(&io_conf);
}
    


// Initialize PCNT functions
void pcntInit() {
   // Initialize PCNT unit
    pcnt_unit_config(&pcnt_config);
    // Set the limit values to watch
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);

    // Set the filter value
    pcnt_set_filter_value(PCNT_TEST_UNIT, FILTER_VALUE);
        
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);


    // Start the PCNT
    pcnt_counter_resume(PCNT_TEST_UNIT);
    
}



unsigned long mil;
unsigned long restmil;
unsigned long last;
unsigned long lastSerial;

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Initialize the Pulse Counter
    configurePullUp();
    pcntInit();

    // Initialize the GPIO for pull-up
//    pinMode(PCNT_INPUT_SIG_IO, INPUT_PULLUP);

    
}

void loop() {
    // Read the pulse count
    
    int16_t count;
    
    int position = 50 * sin(millis() * PI / 300);

    if ( millis() > mil)
    {
      motorA.setPosition(position);
      pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
      pcnt_counter_clear(PCNT_TEST_UNIT);
      countA = count;
      motorA.move(countA);

      if( millis() > lastSerial)
      {
  
        Serial.print(position);    
        Serial.print("  ");     
        Serial.print(countA);    
        Serial.print("  ");     
        Serial.print(motorA.lastDirection);    
        Serial.print("  ");     
        Serial.println(motorA.currentPosition);
  
        
        lastSerial = millis()+ 500;
      }
      
      
      countA = 0;
      restmil = millis() + 1;
      mil = millis() + 500;
    }
    
    if ( millis() > restmil)
    {
      motorA.rest();
      mil = millis() + 2;    
      restmil = millis() + 500;
    }



}
