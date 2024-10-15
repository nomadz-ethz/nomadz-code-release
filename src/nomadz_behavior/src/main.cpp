#include <rclcpp/rclcpp.hpp>

#include "nomadz_behavior/behavior.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  nomadz_behavior::Behavior behavior;
  behavior.run();
  rclcpp::shutdown();
  return 0;
}
