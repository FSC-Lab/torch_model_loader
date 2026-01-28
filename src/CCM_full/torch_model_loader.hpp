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
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>


class TorchModelLoader : public rclcpp::Node{
public:
    explicit TorchModelLoader(const rclcpp::NodeOptions & options);
    virtual ~TorchModelLoader() = default;

    // using InputType = network_loader::msg::ModelInput;
    // using OutputType = network_loader::msg::ModelOutput;
    // using ModelPub = rclcpp::Publisher<OutputType>::SharedPtr;

private:
    static constexpr uint32_t qos = 10u; // quality of service in the publisher and subscriber
    
    
    // Inialize member variables for model inputs
    nav_msgs::msg::Odometry latest_odom_est_;
    px4_msgs::msg::VehicleAttitude latest_vehicle_attitude_;
    std_msgs::msg::Float32MultiArray latest_xref_;
    std_msgs::msg::Float32MultiArray latest_uref_;
    bool CCM_activated_{false};
    
    // torch model related
    void SetupDevice(bool use_cpu = false);
    void LoadModules();
    void GetModelOutputs(const std::vector<torch::jit::IValue> &model_input, std_msgs::msg::Float32MultiArray &output_msg);
    std::vector<torch::jit::IValue> PackInputs(const nav_msgs::msg::Odometry &latest_odom_est_,
                                               const px4_msgs::msg::VehicleAttitude &latest_vehicle_attitude_,
                                               const std_msgs::msg::Float32MultiArray &latest_xref_,
                                               const std_msgs::msg::Float32MultiArray &latest_uref_);
    torch::DeviceType device_type_;
    torch::jit::script::Module module_;
    bool load_successful_ = false; // Flag to indicate if model loading was successful

    //parameter modelList;
    void LoadParameters();
    std::string model_name_;
    std::string model_path_;
    double model_rate_;
    int input_dims_x_;
    int input_dims_xref_;
    int input_dims_uref_;
    int output_dims_;
    double model_period_{0.0};

    // topics related
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr xref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr uref_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr estimator_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr CCM_activated_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr output_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr x_pub_;

    void TimerCallback();

};

#endif // TORCH_MODEL_LOADER_HPP