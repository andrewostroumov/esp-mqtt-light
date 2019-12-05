# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp-mqtt-light)

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "security/ca.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "security/client.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "security/client.key" TEXT)
