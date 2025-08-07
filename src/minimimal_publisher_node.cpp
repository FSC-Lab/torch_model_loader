#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/model/input", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Vector3Stamped();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link";
    message.vector.x = static_cast<double>(0.1*count_);
    message.vector.y = static_cast<double>(0.1*count_ * 2);
    message.vector.z = static_cast<double>(0.1*count_ * 3);

    // RCLCPP_INFO(this->get_logger(), "Publishing: x=%f, y=%f, z=%f",
    //             message.vector.x, message.vector.y, message.vector.z);
    publisher_->publish(message);
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}