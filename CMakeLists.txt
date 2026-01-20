cmake_minimum_required(VERSION 3.18)
project(BNO085)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Find CUDA (if needed)
find_package(CUDAToolkit)
find_package(Open3D REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Fetch sh2 library
include(FetchContent)
FetchContent_Declare(
    sh2
    GIT_REPOSITORY https://github.com/ceva-dsp/sh2.git
    GIT_TAG main  # Fixed: was "mainS"
)
FetchContent_MakeAvailable(sh2)

# Create sh2 library from fetched content
file(GLOB SH2_SOURCES "${sh2_SOURCE_DIR}/*.c")
add_library(sh2_lib STATIC ${SH2_SOURCES})
target_include_directories(sh2_lib PUBLIC ${sh2_SOURCE_DIR})

# Add executable
add_executable(BNO085
    src/main.cpp
    src/I2C_HAL.cpp
    src/I2C_Device.cpp
    src/BNO085.cpp
    src/Visualization.cpp
    src/GUI.cpp
)

# Set include directories for executable
target_include_directories(BNO085 PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${sh2_SOURCE_DIR}  # This points to the fetched content
    ${GPIOD_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(BNO085 PRIVATE
    sh2_lib 
    ${GPIOD_LIBRARIES}
    ${Open3D_LIBRARIES}
)

# If CUDA is available, link CUDA runtime
if(CUDAToolkit_FOUND)
    target_link_libraries(BNO085 PRIVATE CUDA::cudart)
    message(STATUS "Linking with CUDA runtime")
endif()