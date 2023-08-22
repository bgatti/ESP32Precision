//include "esp32-hal-pcnt.h"
#include "driver/pcnt.h"


#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      32000
#define PCNT_L_LIM_VAL     -32000
#define PCNT_THRESH1_VAL    5
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   36  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  4   // Control GPIO HIGH=count up, LOW=count down
#define FILTER_VALUE        26666  // The computed filter value for 3000 Hz

void setup() {
    Serial.begin(115200);

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

void loop() {
    int16_t count;
    pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
    Serial.println(count);

    delay(1000);
}
