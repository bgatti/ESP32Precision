// components/LIS2DW12/test/test_LIS2DW12.c
#include "unity.h"
#include "LIS2DW12.h"

static lis_axis3bit16_t test_accel_data;

TEST_CASE("accel_begin", "[LIS2DW12]")
{
    LIS2DW12StatusTypeDef status = accel_begin();
    TEST_ASSERT_EQUAL(LIS2DW12_STATUS_OK, status);
}

TEST_CASE("test read_accel_raw", "[LIS2DW12]")
{
    LIS2DW12StatusTypeDef status = read_accel_raw(&test_accel_data);
    TEST_ASSERT_EQUAL(LIS2DW12_STATUS_OK, status);
}

TEST_CASE("test set_accel_mode", "[LIS2DW12]")
{
    LIS2DW12StatusTypeDef status = set_accel_mode(LIS2DW12_HIGH_PERFORMANCE_MODE);
    TEST_ASSERT_EQUAL(LIS2DW12_STATUS_OK, status);
}


