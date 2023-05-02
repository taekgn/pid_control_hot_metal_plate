include(../pico_sdk_import.cmake)

add_library(PID_v1 PID_v1.cpp PID_v1.h)
target_sources(pid_ctrl PUBLIC 
               "PID_v1.cpp"
               "PID_v1.h"
               )
target_link_libraries(PID_v1 LINK_PUBLIC
                      pico_stdlib
                      hardware_gpio
                      )