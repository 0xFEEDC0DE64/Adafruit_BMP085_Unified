set(headers
    Adafruit_BMP085_U.h
)

set(sources
    Adafruit_BMP085_U.cpp
)

set(dependencies
    arduino-esp32
    Adafruit_Sensor
    espcpputils
    espchrono
)

idf_component_register(
    INCLUDE_DIRS
        .
    SRCS
        ${headers}
        ${sources}
    REQUIRES
        ${dependencies}
)

target_compile_options(${COMPONENT_TARGET}
    PRIVATE
        -fstack-reuse=all
        -fstack-protector-all
        -Wno-unused-function
        -Wno-deprecated-declarations
        -Wno-missing-field-initializers
        -Wno-parentheses
)
