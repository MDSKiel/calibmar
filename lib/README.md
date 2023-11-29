# COLMAP geomar fork
_(internal fork repo commit id from underwater branch: c63a7255eb8fc676378c25aa9b99118eed6ad388)_
## Updating
To upate the included colmap copy the desired version into the colmap/ subfolder.
Currently also the `colmap` target inside `colmap/CMakeLists.txt` needs to be edited as follows
```cmake
add_library(colmap INTERFACE)
target_link_libraries(colmap INTERFACE ${COLMAP_EXPORT_LIBS})
set(INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
target_include_directories(
    colmap INTERFACE 
    $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>) # <= this line is added
```
since colmap is not consumed from install.