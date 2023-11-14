idf_component_register(
    SRCS hs300x.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)
