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

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=address)
endif()


