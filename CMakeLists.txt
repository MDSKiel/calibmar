cmake_minimum_required(VERSION 3.16)

option(TESTS_ENABLED "Whether to build test binaries" OFF)

project(calibmar
        VERSION 1.0
        LANGUAGES CXX)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/calibmar/version.h.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/src/calibmar/version.h")


add_subdirectory(lib)
add_subdirectory(src/app)
add_subdirectory(src/calibmar)

if(TESTS_ENABLED) 
  enable_testing()
  add_subdirectory(tests)
endif()
