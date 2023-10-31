idf_build_get_property(idf_target IDF_TARGET)

idf_component_register(
    # sources to compile
    SRCS src/zmod4xxx.c HAL/zmod4xxx_hal.c
    INCLUDE_DIRS include HAL lib/esp32s3
    REQUIRES driver i2cdev log
)

idf_build_get_property(build_dir BUILD_DIR)

set(target_name "${idf_target}")
# This sets the link directory for the COMPONENT_LIB. 
# It's specifying a directory where linker libraries for the component will be found
target_link_directories(${COMPONENT_LIB} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/${target_name}")

# adding a prebuilt library for iaq_2nd_gen
set(blob iaq_2nd_gen)
add_prebuilt_library(${blob} "${CMAKE_CURRENT_SOURCE_DIR}/lib/${target_name}/lib_${blob}.a"
                        REQUIRES ${COMPONENT_NAME})

# This links the ${blob} library to the COMPONENT_LIB
message("Linking ${blob} library: ${CMAKE_CURRENT_SOURCE_DIR}/lib/${target_name}/lib_${blob}.a")
target_link_libraries(${COMPONENT_LIB} PUBLIC ${blob})

# adding a prebuilt library for zmod4xxx_cleaning
set(blobcleaning zmod4xxx_cleaning)
add_prebuilt_library(${blobcleaning} "${CMAKE_CURRENT_SOURCE_DIR}/lib/${target_name}/lib_${blobcleaning}.a"
                        REQUIRES ${COMPONENT_NAME})

# This links the ${blob1} library to the COMPONENT_LIB
target_link_libraries(${COMPONENT_LIB} PUBLIC ${blobcleaning})

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

