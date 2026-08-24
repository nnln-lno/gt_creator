#include "gt_creator/gt_creator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto gt_node = std::make_shared<navigation::gtCreator>();

  rclcpp::executors::StaticSingleThreadedExecutor executor;

  executor.add_node(gt_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
