idf_build_get_property(idf_target IDF_TARGET)

idf_component_register(
    # sources to compile
    SRCS src/zmod4xxx.c HAL/zmod4xxx_hal.c
    INCLUDE_DIRS include HAL
    REQUIRES driver i2cdev log
)

idf_build_get_property(build_dir BUILD_DIR)

set(target_name "${idf_target}")
target_link_directories(${COMPONENT_LIB} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/${target_name}")

set(blob iaq_2nd_gen)
add_prebuilt_library(${blob} "${CMAKE_CURRENT_SOURCE_DIR}/lib/${target_name}/lib_${blob}.a"
                        REQUIRES ${COMPONENT_NAME})
target_link_libraries(${COMPONENT_LIB} PUBLIC ${blob})

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

