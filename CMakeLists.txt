cmake_minimum_required(VERSION 3.16)

project(calibmar
        VERSION 1.0
        LANGUAGES CXX)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/calibmar/version.h.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/src/calibmar/version.h")

enable_testing()

add_subdirectory(lib)
add_subdirectory(src/app)
add_subdirectory(src/calibmar)
add_subdirectory(tests)