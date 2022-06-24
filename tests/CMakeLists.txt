find_package(Boost COMPONENTS unit_test_framework REQUIRED)

add_executable(calibmar_tests tests.cpp)
add_subdirectory(calibrators)
add_subdirectory(evaluation EXCLUDE_FROM_ALL)
add_subdirectory(extractors)
add_subdirectory(pose_suggestion)
add_subdirectory(report)
add_subdirectory(utils)

target_include_directories(calibmar_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${Boost_INCLUDE_DIRS})
target_compile_definitions(calibmar_tests PRIVATE "BOOST_TEST_DYN_LINK=1")
target_link_libraries(calibmar_tests PRIVATE calibmar ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
