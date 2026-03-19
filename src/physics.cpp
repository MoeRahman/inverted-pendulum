#include "physics.hpp"
#include <iostream>

Eigen::Vector4d X(0.f, 0.f, 0.f, 0.f)

void test(){
    Eigen::Vector2d pos(0.0, 0.0);
    Eigen::Vector2d vel(1, 2);
    
    pos += vel;
    pos += vel;

    std::cout << "New Position: [" << pos.x() << ", " << pos.y() << "]" << std::endl;
}