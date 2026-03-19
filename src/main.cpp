#include <Eigen/Dense>
#include <iostream>

int main() {
  // 1. Allocate 2D vectors (No manual alloc/free needed!)
  Eigen::Vector2d pos(0.0, 0.0);
  Eigen::Vector2d vel(0.1, 0.2);

  // 2. Update position: pos = pos + vel
  pos += vel;

  // 3. Print values
  std::cout << "New Position: [" << pos.x() << ", " << pos.y() << "]"
            << std::endl;

  // No need to call 'free' - Eigen cleans up automatically!
  return 0;
}
