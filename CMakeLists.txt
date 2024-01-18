cmake_minimum_required(VERSION 3.16)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(CALIBMAR_TESTS_ENABLED "Whether to build test binaries" ON)

project(calibmar
        VERSION 1.0
        LANGUAGES CXX)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/calibmar/version.h.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/src/calibmar/version.h")

add_subdirectory(lib)
add_subdirectory(src/app)
add_subdirectory(src/calibmar)

if(CALIBMAR_TESTS_ENABLED) 
  enable_testing()
  add_subdirectory(tests)
endif()

include(package)