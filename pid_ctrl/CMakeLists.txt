cmake_minimum_required(VERSION 3.25)

include(pico_sdk_import.cmake)

project(pid_ctrl)

pico_sdk_init()
if (TARGET tinyusb_device) 
#Above code will enable serial comms
    add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    )
    #FILE(GLOB PID_v1 pid_lib/*.cpp)
    #add_library(PID_v1 STATIC ${PID_v1})
    add_subdirectory(pid_lib) 
    target_include_directories(PID_v1 PUBLIC pid_lib) # It helps to find header file
    target_link_libraries(${PROJECT_NAME}  
    PID_v1)

    pico_enable_stdio_usb(${PROJECT_NAME} 1)
    #This will enable serial output via USB 1 is enable
    pico_enable_stdio_uart(${PROJECT_NAME} 1)
    #This will enable serial output via UART 0 is disable

    #add_library(PID_v1 PID_v1.cpp) 
    #target_sources(PID_v1 PUBLIC PID_v1.h)

    #Everything when you call library make sure add their name here
    #Order :: Filename, library 1, library 2 and so on
    target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_spi hardware_pwm hardware_irq hardware_uart PID_v1)
    # create map/bin/hex file etc.
    pico_add_extra_outputs(${PROJECT_NAME})
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()