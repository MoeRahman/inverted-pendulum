cmake_minimum_required(VERSION 3.16)
project(inverted_pendulum VERSION 1.0.0)

find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SDL3 REQUIRED)

add_executable(inverted_pendulum 
  src/main.cpp
  src/physics.cpp
)

target_include_directories(inverted_pendulum PRIVATE
  include
  /usr/include/eigen3
)

target_link_libraries(inverted_pendulum PRIVATE 
    OpenGL::GL
    Eigen3::Eigen
    SDL3::SDL3
)

