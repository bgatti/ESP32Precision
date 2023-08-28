# components/LIS2DW12/test/CMakeLists.txt

idf_component_register(
    SRCS "test_LIS2DW12.c"
    INCLUDE_DIRS "."
    REQUIRES unity LIS2DW12
)
