idf_component_register(SRCS "MotorDriver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver espressif__bdc_motor espressif__pid_ctrl esp_timer)

