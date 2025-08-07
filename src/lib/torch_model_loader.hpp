# ifndef TORCH_MODEL_LOADER_HPP
#define TORCH_MODEL_LOADER_HPP

#include <torch/script.h>  // One-stop header.
#include <torch/torch.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <filesystem>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "network_loader/msg/model.hpp"

class TorchModelLoader : public rclcpp::Node{
public:
    explicit TorchModelLoader(const rclcpp::NodeOptions & options);
    virtual ~TorchModelLoader() = default;

    using InputType = geometry_msgs::msg::Vector3Stamped;
    using OutputType = network_loader::msg::Model;
    using ModelPub = rclcpp::Publisher<OutputType>::SharedPtr;

private:
    static constexpr uint32_t qos = 10u; // quality of service in the publisher and subscriber
    
    // torch model related
    void SetupDevice(bool use_cpu = false);
    void LoadModules();
    void GetModelOutputs(torch::Tensor input_tensor, OutputType &output_msg);
    torch::Tensor PackInputs(InputType input_msg);
    torch::DeviceType device_type_;
    torch::jit::script::Module module_;
    bool load_successful_ = false; // Flag to indicate if model loading was successful

    //parameter modelList;
    void LoadParameters();
    std::string model_name_;
    std::string model_path_;
    int input_dims_;
    int output_dims_;

    // topics related
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OutputType>::SharedPtr publisher_;
    void ListenerCallback(const InputType &msg);
    void TimerCallback();

    // listener related
    InputType latest_message_;
    rclcpp::Subscription<InputType>::SharedPtr subscription_;

};

#endif // TORCH_MODEL_LOADER_HPP