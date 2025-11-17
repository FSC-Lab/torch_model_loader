#include "torch_model_loader.hpp"

TorchModelLoader::TorchModelLoader(const rclcpp::NodeOptions & options):
    Node("model_loader", options)
{
    LoadParameters();
    SetupDevice(false); // Default to using GPU if available, otherwise CPU
    LoadModules();

    // Create subscriber to "trajectory_generator" topic with specific uav_prefix
    position_ref_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "uav_0/trajectory_generator/position",
            10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                this->latest_position_ref_ = *msg;
            }
        );
    velocity_ref_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "uav_0/trajectory_generator/velocity",
            10,
            [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
                this->latest_velocity_ref_ = *msg;
            }
        );
    uref_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "uav_0/trajectory_generator/uref",
            10,
            [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
                this->latest_uref_ = *msg;
            }
        );
    estimator_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "uav_0/state_estimator/local_position/odom",
            10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->latest_odom_est_ = *msg;
            }
        );
    CCM_activated_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "uav_0/fsc_autopilot_ros2/CCM_activated", // need to change
            10,
            [this] (const std_msgs::msg::Bool::SharedPtr msg) {
                this->CCM_activated_ = msg->data;
            }
        );
        
    output_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("uav_0/model_loader/thrust_setpoint", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&TorchModelLoader::TimerCallback, this));
}
        

void TorchModelLoader::LoadParameters() {
    // Declare parameters with default values
    this->declare_parameter<std::string>("model_name", "");
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<int>("input_dims_x", -1);
    this->declare_parameter<int>("input_dims_xe", -1);
    this->declare_parameter<int>("input_dims_uref", -1);
    this->declare_parameter<int>("output_dims", -1);
    
    // Get the parameter values
    std::string model_name;
    std::string model_path;
    int input_dims_x;
    int input_dims_xe;
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
    
    if (!this->get_parameter("input_dims_x", input_dims_x)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load input_dims parameter.");
        rclcpp::shutdown();
        std::exit(1);
    }

    if (!this->get_parameter("input_dims_xe", input_dims_xe)) {
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
    
    if (input_dims_x <= 0) {
        RCLCPP_ERROR(this->get_logger(), "input_dims must be positive, got: %d", input_dims_x);
        throw std::runtime_error("Invalid input_dims parameter");
    }

    if (input_dims_xe <= 0) {
        RCLCPP_ERROR(this->get_logger(), "input_dims must be positive, got: %d", input_dims_xe);
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
    RCLCPP_INFO(this->get_logger(), "  Input dims_x: %d", input_dims_x);
    RCLCPP_INFO(this->get_logger(), "  Input dims_xe: %d", input_dims_xe);
    RCLCPP_INFO(this->get_logger(), "  Input dims_uref: %d", input_dims_uref);
    RCLCPP_INFO(this->get_logger(), "  Output dims: %d", output_dims);
    
    // Store as member variables for later use
    model_name_ = model_name;
    model_path_ = model_path;
    input_dims_x_ = input_dims_x;
    input_dims_xe_ = input_dims_xe;
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


std::vector<torch::jit::IValue> TorchModelLoader::PackInputs(const geometry_msgs::msg::Point &latest_position_ref_,
                                                            const geometry_msgs::msg::Vector3 &latest_velocity_ref_,
                                                            const geometry_msgs::msg::Vector3 &latest_uref_,
                                                            const nav_msgs::msg::Odometry &latest_odom_est_)
{
    // Create tensors for each input
    torch::Tensor x = torch::tensor({static_cast<float>(latest_odom_est_.pose.pose.position.x),
                                     static_cast<float>(latest_odom_est_.pose.pose.position.y),
                                     static_cast<float>(latest_odom_est_.pose.pose.position.z),
                                     static_cast<float>(latest_odom_est_.twist.twist.linear.x),
                                     static_cast<float>(latest_odom_est_.twist.twist.linear.y),
                                     static_cast<float>(latest_odom_est_.twist.twist.linear.z)}, torch::kFloat32).reshape({1, 6, 1});

    torch::Tensor xref = torch::tensor({static_cast<float>(latest_position_ref_.x),
                                        static_cast<float>(latest_position_ref_.y),
                                        static_cast<float>(latest_position_ref_.z),
                                        static_cast<float>(latest_velocity_ref_.x),
                                        static_cast<float>(latest_velocity_ref_.y),
                                        static_cast<float>(latest_velocity_ref_.z)}, torch::kFloat32).reshape({1, 6, 1});

    torch::Tensor uref = torch::tensor({static_cast<float>(latest_uref_.x), 
                                        static_cast<float>(latest_uref_.y), 
                                        static_cast<float>(latest_uref_.z)}, torch::kFloat32).reshape({1, 3, 1});

    // Move to CUDA if needed
    if (device_type_ == torch::kCUDA) {
        x = x.to(torch::kCUDA);
        xref = xref.to(torch::kCUDA);
        uref = uref.to(torch::kCUDA);
    }

    // RCLCPP_INFO(this->get_logger(), "Converted input message to tensors");

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
    std::vector<torch::jit::IValue> model_input = PackInputs(latest_position_ref_,
                                                             latest_velocity_ref_,
                                                             latest_uref_,
                                                             latest_odom_est_);

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
