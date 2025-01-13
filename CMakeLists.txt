cmake_minimum_required(VERSION 3.17)

set(SOURCES
        src/TMC2209.cpp
)

set(COMPONENTS
        driver
        gpio
        stepper
        freertos
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS include REQUIRES ${COMPONENTS})
