cmake_minimum_required(VERSION 3.16)
project(inverted_pendulum VERSION 1.0.0)

add_executable(inverted_pendulum 
  src/main.c
  core/physics.c
)

target_include_directories(inverted_pendulum PRIVATE
  include
)

# link the math.h library
target_link_libraries(inverted_pendulum PRIVATE
  m
)

set_target_properties(inverted_pendulum PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Times each source file compilation
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")

# Times the final linking step
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CMAKE_COMMAND} -E time")
