cmake_minimum_required(VERSION 3.16)
project(inverted_pendulum VERSION 1.0.0)

# 1. Find Dependencies
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SDL3 REQUIRED)

# 2. Define Executable
add_executable(inverted_pendulum src/main.cpp)

# 3. Link Libraries
target_link_libraries(inverted_pendulum PRIVATE 
    OpenGL::GL
    Eigen3::Eigen
    SDL3::SDL3
)

