cmake_minimum_required(VERSION 2.8)

# this line is needed only if you use boards with selectable cpu (like mega, pro etc)
# and this line should go before call to "cmake/ArduinoToolchain.cmake"
# this is durty hack and should be fixed somewhen, because it should go to 
# particular cmake subdirectory
set(ARDUINO_CPU atmega2560)

set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake) # Arduino Toolchain

# Set a variable for commands below
set(PROJECT_NAME Repetier)

# Define your project and language
project(${PROJECT_NAME} C CXX)

print_board_list()
print_programmer_list()

# add libraries to project
#link_directories(${ARDUINO_SDK}/libraries)

set(${PROJECT_NAME}_BOARD mega)

# choose compiling for RF1000 or RF2000
option(RF1000 "Build for the RF1000 printer" OFF)
option(RF2000 "Build for the RF2000 printer" OFF)
option(RF2000v2 "Build for the RF2000v2 printer" OFF)

if( (RF1000 AND RF2000) OR (RF1000 AND RF2000v2) OR (RF2000 AND RF2000v2) OR (RF1000 AND RF2000 AND RF2000v2) OR (NOT RF1000 AND NOT RF2000 AND NOT RF2000v2) )
  message(FATAL_ERROR "Please select to build either for RF1000 or RF2000 / RF2000v2 by setting either '-DRF1000=ON' or '-DRF2000=ON' or '-DRF2000v2=ON'.")
endif()
if(RF1000)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMOTHERBOARD=DEVICE_TYPE_RF1000")
elseif(RF2000)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMOTHERBOARD=DEVICE_TYPE_RF2000")
elseif(RF2000v2)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMOTHERBOARD=DEVICE_TYPE_RF2000v2")
endif()

# Define the source code
setup_arduino_sketch(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/Repetier.ino SKETCHSRC)
aux_source_directory(${CMAKE_SOURCE_DIR} repetier_sources)
include_directories(${CMAKE_SOURCE_DIR})
set(${PROJECT_NAME}_SRCS ${SKETCHSRC} ${repetier_sources})

# Command to generate code arduino firmware (.hex file)
generate_arduino_firmware(${PROJECT_NAME})

