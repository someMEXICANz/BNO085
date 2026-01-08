cmake_minimum_required(VERSION 3.18)
project(BNO085)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find CUDA (if needed)
find_package(CUDAToolkit)

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
)

# Set include directories for executable
target_include_directories(BNO085 PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${sh2_SOURCE_DIR}  # This points to the fetched content
)

# Link libraries
target_link_libraries(BNO085 PRIVATE
    sh2_lib  # Added: link the sh2 library!
)

# If CUDA is available, link CUDA runtime
if(CUDAToolkit_FOUND)
    target_link_libraries(BNO085 PRIVATE CUDA::cudart)
    message(STATUS "Linking with CUDA runtime")
endif()