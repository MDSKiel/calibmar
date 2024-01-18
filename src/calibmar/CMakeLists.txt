find_package(OpenCV 4 REQUIRED)

add_library(calibmar)
add_subdirectory(calibrators)
add_subdirectory(core)
add_subdirectory(extractors)
add_subdirectory(pose_suggestion)
add_subdirectory(readers)

target_link_libraries(calibmar PUBLIC colmap ${OpenCV_LIBS})
target_include_directories(calibmar PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/.. ${OpenCV_INCLUDE_DIRS})
target_compile_features(calibmar PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(calibmar PUBLIC "/bigobj")
endif()

if(CUDA_ENABLED) # to also use the colmap cuda enabled option
    target_compile_definitions(calibmar PUBLIC COLMAP_CUDA_ENABLED)
endif()