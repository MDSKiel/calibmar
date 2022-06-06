# COLMAP geomar fork
_(internal fork repo commit id: 9482ee21439ff7c37c149dca15a2005f6bef6e47)_
## Updating
To upate the included colmap copy the desired version into the colmap/ subfolder.
Currently also the `colmap` target inside `colmap/src/CMakeLists.txt` needs to be edited by adding
```cmake
target_include_directories(colmap PUBLIC    
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib
    ${CMAKE_CURRENT_SOURCE_DIR}    
    ${COLMAP_INCLUDE_DIRS}
)
target_include_directories(colmap INTERFACE
    ${PROJECT_SOURCE_DIR}/.. #so we can include from external via <colmap/src/...>)
)
target_link_directories(colmap PUBLIC ${COLMAP_LINK_DIRS})

if(OpenMP_CXX_FOUND)
list(APPEND COLMAP_EXTERNAL_LIBRARIES OpenMP::OpenMP_CXX)
endif()

if(IS_MSVC)
    target_compile_options(colmap PUBLIC "/bigobj")
endif()
```
so that the target properly propagates its properties.

The `colmap/cmake/FindFreeImage.cmake` has also been adapted to support finding debug version of the freeimage library.
This is useful when building Debug with vcpkg. This is kept optional since under Linux freeimaged is not available.