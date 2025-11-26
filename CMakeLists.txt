cmake_minimum_required(VERSION 3.16)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(CALIBMAR_TESTS_ENABLED "Whether to build test binaries" ON)

project(calibmar
        VERSION 1.0.0
        LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to Release, for debug builds use '-DCMAKE_BUILD_TYPE=Debug'.")
  set(CMAKE_BUILD_TYPE "Release")
endif()        

# Install application menu entry under Linux.
if(UNIX AND NOT APPLE)
    install(FILES "cmake/calibmar.desktop" DESTINATION "share/applications")
endif()

add_subdirectory(lib)
add_subdirectory(src/app)
add_subdirectory(src/calibmar)

if(CALIBMAR_TESTS_ENABLED) 
  enable_testing()
  add_subdirectory(tests)
endif()

include(package)