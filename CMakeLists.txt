idf_component_register(
    SRCS
        "src/M5PM1.cpp"
    INCLUDE_DIRS
        "src"
    REQUIRES
        "driver"
    # NOTE: espressif__i2c_bus dependency removed to avoid conflict with M5GFX
)
