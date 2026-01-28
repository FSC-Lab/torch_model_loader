#include "torch_model_loader.hpp"

inline rclcpp::QoS qos_best_effort_transient_local()
{
    return rclcpp::QoS(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

inline Eigen::Quaterniond getNEDqFromENUq(const Eigen::Quaterniond& in) {
  //
  const Eigen::Matrix3d Rie =
      (Eigen::Matrix3d() << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0)
          .finished();
  const Eigen::Matrix3d Rbv =
      (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0)
          .finished();
  // ned -> enu conversion
  return Eigen::Quaterniond(Rie) * in * Eigen::Quaterniond(Rbv);
}

TorchModelLoader::TorchModelLoader(const rclcpp::NodeOptions & options):
    Node("model_loader", options)
{
    LoadParameters();
    SetupDevice(false); // Default to using GPU if available, otherwise CPU
    LoadModules();
    
    latest_xref_.data.resize(15, 0.0f);
    latest_uref_.data.resize(4, 0.0f);
    
    // Create subscriber to "trajectory_generator" topic with specific uav_prefix
    estimator_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "state_estimator/local_position/odom",
            qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->latest_odom_est_ = *msg;
            }
        );
    
    vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "fmu/out/vehicle_attitude",
            qos_best_effort_transient_local(),
            [this](const px4_msgs::msg::VehicleAttitude::ConstSharedPtr& msg) {
                this->latest_vehicle_attitude_ = *msg;
            }    
        );

    xref_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "trajectory_generator/xref",
            qos,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                this->latest_xref_ = *msg;
            }
        );

    uref_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "trajectory_generator/uref",
            qos,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                this->latest_uref_ = *msg;
            }
        );
    
    CCM_activated_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "fsc_autopilot_ros2/CCM_activated", 
            qos,
            [this] (const std_msgs::msg::Bool::SharedPtr msg) {
                this->CCM_activated_ = msg->data;
            }
        );

    output_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("model_loader/rate_setpoint", 10);
    x_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("model_loader/x", 10);

    model_period_ = 1/model_rate_;  // seconds per cycle 
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(model_period_), std::bind(&TorchModelLoader::TimerCallback, this));
}
        

void TorchModelLoader::LoadParameters() {
    // Declare parameters with default values
    this->declare_parameter<std::string>("model_name", "");
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<double>("model_rate", -1.0);
    this->declare_parameter<int>("input_dims_x", -1);
    this->declare_parameter<int>("input_dims_xref", -1);
    this->declare_parameter<int>("input_dims_uref", -1);
    this->declare_parameter<int>("output_dims", -1);
    
    // Get the parameter values
    std::string model_name;
    std::string model_path;
    double model_rate;
    int input_dims_x;
    int input_dims_xref;
    int input_dims_uref;
    int output_dims;
    
    if (!this->get_parameter("model_name", model_name)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model_name parameter.");
        rclcpp::shutdown();
        std::exit(1);
    }
    
    if (!this->get_parameter("model_path", model_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model_path parameter.");
        rclcpp::shutdown();
        std::exit(1);
    }

    if (!this->get_parameter("model_rate", model_rate)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model_rate parameter.");
        rclcpp::shutdown();
        std::exit(1);
    }    
    
    if (!this->get_parameter("input_dims_x", input_dims_x)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load input_dims parameter.");
        rclcpp::shutdown();
        std::exit(1);
    }

    if (!this->get_parameter("input_dims_xref", input_dims_xref)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load input_dims parameter.");
        rclcpp::shutdown();
        std::exit(1);
    }

    if (!this->get_parameter("input_dims_uref", input_dims_uref)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load input_dims parameter.");
        rclcpp::shutdown();
        std::exit(1);
    }
    
    if (!this->get_parameter("output_dims", output_dims)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load output_dims parameter.");
        rclcpp::shutdown();
        std::exit(1);
    }
    
    // Validate parameter values
    if (model_name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "model_name parameter cannot be empty");
        throw std::runtime_error("Invalid model_name parameter");
    }
    
    if (model_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "model_path parameter cannot be empty");
        throw std::runtime_error("Invalid model_path parameter");
    }

    if (model_rate < 100.0) {
        RCLCPP_ERROR(this->get_logger(), "model_rate parameter must be larger than 100, got: %f", model_rate);
        throw std::runtime_error("Invalid model_rate parameter");
    }    
    
    if (input_dims_x <= 0) {
        RCLCPP_ERROR(this->get_logger(), "input_dims must be positive, got: %d", input_dims_x);
        throw std::runtime_error("Invalid input_dims parameter");
    }

    if (input_dims_xref <= 0) {
        RCLCPP_ERROR(this->get_logger(), "input_dims must be positive, got: %d", input_dims_xref);
        throw std::runtime_error("Invalid input_dims parameter");
    }

    if (input_dims_uref <= 0) {
        RCLCPP_ERROR(this->get_logger(), "input_dims must be positive, got: %d", input_dims_uref);
        throw std::runtime_error("Invalid input_dims parameter");
    }
    
    if (output_dims <= 0) {
        RCLCPP_ERROR(this->get_logger(), "output_dims must be positive, got: %d", output_dims);
        throw std::runtime_error("Invalid output_dims parameter");
    }
    
    // Log the loaded parameters
    RCLCPP_INFO(this->get_logger(), "Loaded model configuration:");
    RCLCPP_INFO(this->get_logger(), "  Model name: %s", model_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  Model path: %s", model_path.c_str());
    RCLCPP_INFO(this->get_logger(), "  Model rate: %f", model_rate);    
    RCLCPP_INFO(this->get_logger(), "  Input dims_x: %d", input_dims_x);
    RCLCPP_INFO(this->get_logger(), "  Input dims_xref: %d", input_dims_xref);
    RCLCPP_INFO(this->get_logger(), "  Input dims_uref: %d", input_dims_uref);
    RCLCPP_INFO(this->get_logger(), "  Output dims: %d", output_dims);
    
    // Store as member variables for later use
    model_name_ = model_name;
    model_path_ = model_path;
    model_rate_ = model_rate;
    input_dims_x_ = input_dims_x;
    input_dims_xref_ = input_dims_xref;
    input_dims_uref_ = input_dims_uref;
    output_dims_ = output_dims;
}


void TorchModelLoader::SetupDevice(bool use_cpu)
{
    // Set the device type
    if (use_cpu) {
        RCLCPP_INFO(this->get_logger(), "Using CPU for model loading.");
        device_type_ = torch::kCPU;
    } else {
        if (torch::cuda::is_available()) {
            RCLCPP_INFO(this->get_logger(), "CUDA is available!");
            device_type_ = torch::kCUDA;
            RCLCPP_INFO(this->get_logger(), "Using CUDA for model loading.");
        } else {
            device_type_ = torch::kCPU;
            RCLCPP_INFO(this->get_logger(), "CUDA is not available, using CPU for model loading.");
        }
    }
}

void TorchModelLoader::LoadModules()
{
    RCLCPP_INFO(this->get_logger(), "Start loading");
    std::string device_prefix = (device_type_ == torch::kCUDA) ? "cuda" : "cpu";
    std::string model_path = model_path_ + model_name_ + "_" + device_prefix + ".pt";  // Assuming models are stored in a 'models' directory

    try {
        module_ = torch::jit::load(model_path);
        torch::Device device(device_type_, 0);
        module_.to(device);
        module_.eval();
        RCLCPP_INFO(this->get_logger(), "Loaded model successfully: %s", model_path.c_str());
        load_successful_ = true;
    } catch (const c10::Error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model %s: %s", model_path.c_str(), e.what());
        load_successful_ = false;
    }
    
}


void TorchModelLoader::GetModelOutputs(const std::vector<torch::jit::IValue> &model_input, std_msgs::msg::Float32MultiArray &output_msg)
{ 
    torch::jit::IValue model_output = module_.forward(model_input);
    torch::Tensor output_tensor = model_output.toTensor();
    if (device_type_ == torch::kCUDA) {
        torch::Device cpu_device_(torch::kCPU, 0);
        output_tensor = output_tensor.to(cpu_device_);
    }
    output_tensor = output_tensor.contiguous();
    float* data_ptr = output_tensor.data_ptr<float>();
    // Check if output dimensions match expected size
    if (output_tensor.numel() != output_dims_) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Model output size mismatch. Expected %d, got %ld", 
                     output_dims_, 
                     output_tensor.numel());
        return;
    }

    // Copy data to the ROS message
    output_msg.data.resize(output_dims_);
    for (int i = 0; i < output_dims_; ++i) {
        output_msg.data[i] = data_ptr[i];
    }

    // // Set the header timestamp
    // output_msg.header.stamp = this->get_clock()->now();
    // output_msg.header.frame_id = "base_link"; // or whatever frame is appropriate
}


std::vector<torch::jit::IValue> TorchModelLoader::PackInputs(const nav_msgs::msg::Odometry &latest_odom_est_,
                                                             const px4_msgs::msg::VehicleAttitude &latest_vehicle_attitude_, 
                                                             const std_msgs::msg::Float32MultiArray &latest_xref_, 
                                                             const std_msgs::msg::Float32MultiArray &latest_uref_)
{
    // Create tensors for each input
    torch::Tensor pos = torch::tensor({
        static_cast<float>(latest_odom_est_.pose.pose.position.x),
        static_cast<float>(latest_odom_est_.pose.pose.position.y),
        static_cast<float>(latest_odom_est_.pose.pose.position.z)
    }, torch::kFloat32).view({1, 3, 1});

    torch::Tensor vel = torch::tensor({
        static_cast<float>(latest_odom_est_.twist.twist.linear.x),
        static_cast<float>(latest_odom_est_.twist.twist.linear.y),
        static_cast<float>(latest_odom_est_.twist.twist.linear.z)
    }, torch::kFloat32).view({1, 3, 1});   
    
    // Quaternion from state estimator
    Eigen::Quaterniond q_odom(latest_odom_est_.pose.pose.orientation.w, latest_odom_est_.pose.pose.orientation.x, 
        latest_odom_est_.pose.pose.orientation.y, latest_odom_est_.pose.pose.orientation.z);
    q_odom.normalize();

    // Use quaternion from pixhawk (PX4 quaternion in NED frame, [w, x, y, z])!!!
    Eigen::Quaterniond q_ned(latest_vehicle_attitude_.q[0], latest_vehicle_attitude_.q[1], 
                             latest_vehicle_attitude_.q[2], latest_vehicle_attitude_.q[3]);
    Eigen::Quaterniond q_enu = getNEDqFromENUq(q_ned);  
    q_enu.normalize(); // Normalize for safety

    // Rotation from FLU to ENU
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_IB = q_enu.toRotationMatrix().cast<float>();   // q_enu (from PX4); q_odom (from state estimator)
    torch::Tensor att = (torch::from_blob(R_IB.data(), {3, 3}, torch::kFloat32).clone()).reshape({1, 9, 1});

    // Stack everything together into a message
    torch::Tensor x = torch::cat({pos, vel, att}, 1).view({1, input_dims_x_, 1});
    torch::Tensor xref = torch::tensor(latest_xref_.data, torch::kFloat32).view({1, input_dims_xref_, 1});
    torch::Tensor uref = torch::tensor(latest_uref_.data, torch::kFloat32).view({1, input_dims_uref_, 1});

    // Move to CUDA if needed
    if (device_type_ == torch::kCUDA) {
        x = x.to(torch::kCUDA);
        xref = xref.to(torch::kCUDA);
        uref = uref.to(torch::kCUDA);
    }

    // RCLCPP_INFO(this->get_logger(), "Converted input message to tensors");

    // Publish x as current state
    torch::Tensor x_ten = x;
    if (x_ten.device().is_cuda()) {
        x_ten = x_ten.to(torch::kCPU);
    }
    x_ten = x_ten.contiguous();

    std_msgs::msg::Float32MultiArray x_msg;
    const float* data = x_ten.data_ptr<float>();

    x_msg.data.resize(x_ten.numel());
    for (int i = 0; i < x_ten.numel(); ++i) {
        x_msg.data[i] = data[i];
    }
    x_pub_->publish(x_msg);

    // Return as a vector for forward()
    return {x, xref, uref};
}


void TorchModelLoader::TimerCallback(){

    if (load_successful_)
    {    
    // if (!CCM_activated_)
    // {
    //     return;
    // }
    
    // Pack inputs from the ROS message
    std::vector<torch::jit::IValue> model_input = PackInputs(latest_odom_est_,
                                                             latest_vehicle_attitude_,
                                                             latest_xref_,
                                                             latest_uref_);

    // Get model outputs
    std_msgs::msg::Float32MultiArray output_msg;
    GetModelOutputs(model_input, output_msg);
    
    // Publish the output message
    output_pub_->publish(output_msg);

    } else 
    {
        RCLCPP_INFO(this->get_logger(), "Model loading failed");
    }
}
