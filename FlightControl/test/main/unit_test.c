/* Example test application for testable component.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void print_banner(const char* text);

void app_main(void)
{

    //delay for 500ms for usb connection
    vTaskDelay(pdMS_TO_TICKS(500));
    /* These are the different ways of running registered tests.
     * In practice, only one of them is usually needed.
     *
     * UNITY_BEGIN() and UNITY_END() calls tell Unity to print a summary
     * (number of tests executed/failed/ignored) of tests executed between these calls.
     */
    print_banner("Executing one test by its name");
    
    // UNITY_BEGIN();
    // unity_run_test_by_name("accel_begin");
    // unity_run_test_by_name("test read_accel_raw");
    // unity_run_test_by_name("test set_accel_mode");
    // UNITY_END();


    
    print_banner("Executing IMU test by its name");
    UNITY_BEGIN();
    // Add your IMU tests by their name
//    unity_run_test_by_name("IMU_Init");
    // unity_run_test_by_name("IMU_Update");
    // unity_run_test_by_name("IMU_Deinit");
    UNITY_END();


    print_banner("Executing MOTOR test by its name");
    UNITY_BEGIN();
        unity_run_test_by_name("MotorDriver_Init");
        unity_run_test_by_name("MotorDriver_Update_Position");
        // unity_run_test_by_name("MotorDriver_DeInit");
    UNITY_END();


   // Guidance Tests
    print_banner("Executing Guidance tests by their names");
    UNITY_BEGIN();
        unity_run_test_by_name("Guidance_Init");
    //    unity_run_test_by_name("ComputeMotorControl_Test");
//        unity_run_test_by_name("Guidance_Loop");

    UNITY_END();



    print_banner("Test Completed");

    
//    print_banner("Starting interactive test menu");
    /* This function will not return, and will be busy waiting for UART input.
     * Make sure that task watchdog is disabled if you use this function.
     */
    //unity_run_menu();
}

static void print_banner(const char* text)
{
    printf("\n#### %s #####\n\n", text);
}
