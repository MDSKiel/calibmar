# Calibmar

Calibmar is a camera and underwater housing calibration tool.

Features include:
- Camera housing calibration based on '[Refractive Geometry for Underwater Domes](https://doi.org/10.1016/j.isprsjprs.2021.11.006)'
	- with a Flat Port model from '[Refractive Calibration of Underwater Cameras](https://doi.org/10.1007/978-3-642-33715-4_61)'
- Calibration guidance implementation of '[Calibration Wizard](https://doi.org/10.1109/iccv.2019.00158)'
- [OpenCV](https://docs.opencv.org/4.5.5/d9/d0c/group__calib3d.html#details) based camera calibration
- [COLMAP](https://colmap.github.io/) compliant camera models
-----------------
## Build from Source
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
        libsuitesparse-dev \
        libfreeimage-dev \
        libmetis-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libglew-dev \
        qtbase5-dev \
        libqt5opengl5-dev \
        libcgal-dev \
		libopencv-dev

Install Ceres Solver <http://ceres-solver.org/>:

    sudo apt-get install libatlas-base-dev libsuitesparse-dev
    git clone https://ceres-solver.googlesource.com/ceres-solver
    cd ceres-solver
    git checkout 2.1.0
    mkdir build
    cd build
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
    make -j
    sudo make install
	
Configure and compile Calibmar:

	cd path/to/calibmar
    mkdir build
    cd build
    cmake .. -DCUDA_ENABLED=OFF
    cmake --build . --config Release

### Windows
For Windows it is recommended to use [vcpkg](https://github.com/microsoft/vcpkg).

    git clone https://github.com/microsoft/vcpkg
    cd vcpkg
    ./bootstrap-vcpkg.sh
    ./vcpkg install colmap:x64-windows
    ./vcpkg install opencv[contrib]:x64-windows

This will take a while.

Configure and compile Calibmar:

	cd path/to/calibmar
    mkdir build
    cd build
    cmake .. -DCMAKE_TOOLCHAIN_FILE=path/to/vcpkg/scripts/buildsystems/vcpkg.cmake -DCUDA_ENABLED=OFF
    cmake --build . --config Release

For VCPKG-Windows, CMake Presets are available which are supported by several IDEs. They expect the environment variable `VCPKG_ROOT` to point to the vcpkg root directory.