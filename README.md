# Calibmar

Calibmar is a camera and underwater housing calibration tool.

Features include:
- Camera housing calibration based on '[Refractive Geometry for Underwater Domes](https://doi.org/10.1016/j.isprsjprs.2021.11.006)'
	- with a Flat Port model from '[Refractive Calibration of Underwater Cameras](https://doi.org/10.1007/978-3-642-33715-4_61)'
- Camera and stereo camera calibration 
- Calibration guidance implementation of '[Calibration Wizard](https://doi.org/10.1109/iccv.2019.00158)'
- [COLMAP](https://colmap.github.io/) compliant camera models

## Install

Binaries for Windows and Linux are available at https://cau-git.rz.uni-kiel.de/inf-ag-koeser/calibmar/-/releases.

## Build from Source

### CUDA

To build with CUDA support install the latest CUDA from NVIDIA's homepage.

You will also need to specify a CUDA architecture during cmake configuration as e.g. `-DCMAKE_CUDA_ARCHITECTURES=native`.

There is a known error for Windows, when installing CUDA together with Visual Studio Build Tools (as opposed to Visual Studio IDE), where CUDA does not properly integrate into the build tools and some files have to be manually copied.

### Linux

The following build has been tested under Ubuntu 22.04.

Dependencies from default Ubuntu repositories:

    sudo apt-get install \
        git \
        cmake \
        build-essential \
        libboost-program-options-dev \
        libboost-filesystem-dev \
        libboost-graph-dev \
        libboost-system-dev \
        libboost-test-dev \
        libeigen3-dev \
        libflann-dev \
        libfreeimage-dev \
        libmetis-dev \
        libgoogle-glog-dev \
        libglew-dev \
        libsqlite3-dev \
        qtbase5-dev \
        libqt5opengl5-dev \
        libcgal-dev \
        libceres-dev \
        libopencv-dev

Configure and compile Calibmar:

	cd path/to/calibmar
    mkdir build
    cd build
    cmake .. -GNinja
    ninja

### Windows

For Windows it is recommended to use [vcpkg](https://github.com/microsoft/vcpkg). You will also need Visual Studio Build Tools.

    git clone https://github.com/microsoft/vcpkg
    cd vcpkg
    bootstrap-vcpkg.sh
    vcpkg install colmap[cuda]:x64-windows
    vcpkg install opencv[contrib]:x64-windows

This will take a while.

Configure and compile Calibmar:

	cd path/to/calibmar
    mkdir build
    cd build
    cmake .. -DCMAKE_TOOLCHAIN_FILE=path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
    cmake --build . --config Release

CMake Presets are available which are supported by several IDEs. Presets for windows expect the environment variable `VCPKG_ROOT` to point to the vcpkg root directory.