idf_component_register(SRCS "testIMU.c"
                    INCLUDE_DIRS "."
                    REQUIRES cmock IMU esp_timer
                    )