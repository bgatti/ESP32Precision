// components/LIS2DW12/test/test_LIS2DW12.c
#include "unity.h"
#include "LIS2DW12.h"

static axis3bit16_t test_accel_data;

TEST_CASE("accel_begin", "[LIS2DW12]")
{
    LIS2DW12StatusTypeDef status = accel_begin();
    TEST_ASSERT_EQUAL(LIS2DW12_STATUS_OK, status);
}

void setUp(void) {
    // Code to be executed before each test
}

void tearDown(void) {
    // Code to be executed after each test
}

void test_accel_begin(void) {
    LIS2DW12StatusTypeDef status = accel_begin();
    TEST_ASSERT_EQUAL(LIS2DW12_STATUS_OK, status);
}

void test_read_accel_raw(void) {
    LIS2DW12StatusTypeDef status = read_accel_raw(&test_accel_data);
    TEST_ASSERT_EQUAL(LIS2DW12_STATUS_OK, status);
}

void test_set_accel_mode(void) {
    LIS2DW12StatusTypeDef status = set_accel_mode(LIS2DW12_HIGH_PERFORMANCE_MODE);
    TEST_ASSERT_EQUAL(LIS2DW12_STATUS_OK, status);
}



int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_accel_begin);
    RUN_TEST(test_read_accel_raw);
    RUN_TEST(test_set_accel_mode);

    return UNITY_END();
}
