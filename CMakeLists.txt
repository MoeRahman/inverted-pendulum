cmake_minimum_required(VERSION 3.16)
project(inverted_pendulum VERSION 1.0.0)

add_executable(inverted_pendulum 
  src/main.c
  core/physics.c
  core/controller.c
)

target_include_directories(inverted_pendulum PRIVATE
  include
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()


