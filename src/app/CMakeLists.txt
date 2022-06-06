find_package(Qt5 5.14 COMPONENTS Core Gui Widgets REQUIRED)
find_package(OpenCV 4 REQUIRED)

add_executable(calibmar_app WIN32 main.cpp)
add_subdirectory(ui)

set_target_properties(calibmar_app PROPERTIES OUTPUT_NAME calibmar)

target_include_directories(calibmar_app PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(calibmar_app PRIVATE    
    calibmar
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${OpenCV_LIBS}
)

install(TARGETS calibmar_app)
