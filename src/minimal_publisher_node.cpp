#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "network_loader/msg/model_input.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<network_loader::msg::ModelInput>("/model/input", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Create message of your custom InputType
    network_loader::msg::ModelInput message;

    // Fill header
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link";

    // Example data for x, xe, uref
    for (int i = 0; i < 9; ++i) {
      message.x[i]  = 0.1f * static_cast<float>(count_ + i);
      message.xe[i] = 0.2f * static_cast<float>(count_ + i);
    }

    for (int i = 0; i < 4; ++i) {
      message.uref[i] = 0.05f * static_cast<float>(count_ + i); 
    }

    // RCLCPP_INFO(this->get_logger(), "Publishing model input #%zu", count_);

    publisher_->publish(message);
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<network_loader::msg::ModelInput>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
