# ifndef TORCH_MODEL_LOADER_HPP
#define TORCH_MODEL_LOADER_HPP

#include <torch/script.h>  // One-stop header.
#include <torch/torch.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <filesystem>
#include <Eigen/Dense>
//#include <vector>
//#include <array>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "network_loader/msg/model_input.hpp"
#include "network_loader/msg/model_output.hpp"

class TorchModelLoader : public rclcpp::Node{
public:
    explicit TorchModelLoader(const rclcpp::NodeOptions & options);
    virtual ~TorchModelLoader() = default;

    using InputType = network_loader::msg::ModelInput;
    using OutputType = network_loader::msg::ModelOutput;
    using ModelPub = rclcpp::Publisher<OutputType>::SharedPtr;

private:
    static constexpr uint32_t qos = 10u; // quality of service in the publisher and subscriber
    
    // torch model related
    void SetupDevice(bool use_cpu = false);
    void LoadModules();
    void GetModelOutputs(const std::vector<torch::jit::IValue> &model_input, OutputType &output_msg);
    std::vector<torch::jit::IValue> PackInputs(const InputType &input_msg);
    torch::DeviceType device_type_;
    torch::jit::script::Module module_;
    bool load_successful_ = false; // Flag to indicate if model loading was successful

    //parameter modelList;
    void LoadParameters();
    std::string model_name_;
    std::string model_path_;
    int input_dims_0_;
    int input_dims_1_;
    int input_dims_2_;
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