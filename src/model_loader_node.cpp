#include <vector>
#include <array>
#include <string>

#include "torch_model_loader.hpp"
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorchModelLoader>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}