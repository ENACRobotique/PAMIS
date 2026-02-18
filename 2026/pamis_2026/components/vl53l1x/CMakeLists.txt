idf_component_register(SRCS "core/VL53L1X_api.c" "core/VL53L1X_calibration.c" "platform/vl53l1_platform.c"
                    INCLUDE_DIRS "core" "platform"
                    REQUIRES driver)
