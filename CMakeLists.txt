#================================================================================
cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE cmake/arm-none-eabi-gcc.cmake)


#================================================================================
project(CW_tele LANGUAGES CXX C ASM)
set(DEVICE stm32f303vc)

include(libopencm3)


#================================================================================
add_executable(${PROJECT_NAME}
     src/main.cpp
     SPI_master/SPI.cpp
     TIMER_setup/timer.cpp
     NRF24_master/RF24.cpp
)

target_include_directories( ${PROJECT_NAME}
PRIVATE ${CMAKE_CURRENT_LIST_DIR} )

target_link_libraries(${PROJECT_NAME}
     PRIVATE libopencm3
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
     COMMAND cp ${PROJECT_NAME} ../firmware
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
     COMMAND ${OBJDUMP_UTIL} -d ${PROJECT_NAME} > ${PROJECT_NAME}.lst
)


#add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  #   COMMAND cp compile_commands.json ../compile_commands.json
#)


set_target_properties(${PROJECT_NAME}
     PROPERTIES
     CXX_STANDARD 17
     CXX_STANDARD_REQUIRED YES
     CXX_EXTENSIONS NO
)


#================================================================================
set(PROG stlink)
string(CONCAT PROG_TARGET ${LOCM3_DEVICE_SERIE} x)

include(oocd_flash_script)

install(CODE ${OOCD_FLASH_SCRIPT})

