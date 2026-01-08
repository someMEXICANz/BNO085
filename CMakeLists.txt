
cmake_minimum_required(VERSION 3.18)
project(BNO085)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Open3D
find_package(Open3D REQUIRED)

# Find CUDA (if Open3D was built with CUDA)
find_package(CUDAToolkit)


include_directories(CameraTest PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)


# Add executable
add_executable(BNO085
    src/main.cpp
    src/I2C_HAL.cpp
    src/I2C_Device.cpp
    src/BNO085.cpp

)


# Link Open3D
target_link_libraries(BNO085 Open3D::Open3D)
target_include_directories(BNO085 PRIVATE ${Open3D_INCLUDE_DIRS})

# If CUDA is available, link CUDA runtime
if(CUDAToolkit_FOUND)
    target_link_libraries(BNO085 CUDA::cudart)
    message(STATUS "Linking with CUDA runtime")
endif()