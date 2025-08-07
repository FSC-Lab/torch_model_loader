#include "torch_model_loader.hpp"

TorchModelLoader::TorchModelLoader(const rclcpp::NodeOptions & options):
    Node("model_loader", options)
{
    LoadParameters();
    SetupDevice(false); // Default to using GPU if available, otherwise CPU
    LoadModules();

    // Create subscriber to "minimal_publisher" topic
    subscription_ = this->create_subscription<InputType>(
            "/model/input",
            10,
            std::bind(&TorchModelLoader::ListenerCallback, this, std::placeholders::_1)
        );
    publisher_ = this->create_publisher<OutputType>("/model/output", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&TorchModelLoader::TimerCallback, this));
}
        

void TorchModelLoader::LoadParameters() {
    // Declare parameters with default values
    this->declare_parameter<std::string>("model_name", "");
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<int>("input_dims", -1);
    this->declare_parameter<int>("output_dims", -1);
    
    // Get the parameter values
    std::string model_name;
    std::string model_path;
    int input_dims;
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
    
    if (!this->get_parameter("input_dims", input_dims)) {
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
    
    if (input_dims <= 0) {
        RCLCPP_ERROR(this->get_logger(), "input_dims must be positive, got: %d", input_dims);
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
    RCLCPP_INFO(this->get_logger(), "  Input dims: %d", input_dims);
    RCLCPP_INFO(this->get_logger(), "  Output dims: %d", output_dims);
    
    // Store as member variables for later use
    model_name_ = model_name;
    model_path_ = model_path;
    input_dims_ = input_dims;
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

void TorchModelLoader::GetModelOutputs(torch::Tensor input_tensor, OutputType &output_msg)
{ 
    std::vector<torch::jit::IValue> model_input;
    model_input.push_back(input_tensor);
    torch::jit::IValue model_output = module_.forward(model_input);
    torch::Tensor output_tensor = model_output.toTensor();
    if (device_type_ == torch::kCUDA) {
        torch::Device cpu_device_(torch::kCPU, 0);
        output_tensor = output_tensor.to(cpu_device_);
    }
    output_tensor = output_tensor.contiguous();
    float* data_ptr = output_tensor.data_ptr<float>();
    // Check if output dimensions match expected size
    if (output_tensor.numel() != 5) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Model output size mismatch. Expected 5, got %ld", 
                     output_tensor.numel());
        return;
    }

    // Copy data to the ROS message
    for (int i = 0; i < 5; ++i) {
        output_msg.model_output[i] = data_ptr[i];
    }

    // Set the header timestamp
    output_msg.header.stamp = this->get_clock()->now();
    output_msg.header.frame_id = "base_link"; // or whatever frame is appropriate
}

torch::Tensor TorchModelLoader::PackInputs(InputType input_msg)
{
    // Create data vector directly
    std::vector<float> input_data(input_dims_, 0.0f);
    input_data[0] = static_cast<float>(input_msg.vector.x);
    input_data[1] = static_cast<float>(input_msg.vector.y);
    input_data[2] = static_cast<float>(input_msg.vector.z);
    input_data[3] = static_cast<float>(input_msg.vector.y);
    input_data[4] = static_cast<float>(input_msg.vector.x);

    // Create tensor directly from data
    torch::Tensor input_tensor = torch::tensor(input_data, torch::dtype(torch::kFloat32))
                                .view({1, input_dims_}); // clone() makes it own the data
    
    RCLCPP_INFO(this->get_logger(), 
                        "Convert the input message to tensor");                   
    // Move to device if needed
    if (device_type_ == torch::kCUDA) {
        torch::Device device(device_type_, 0);
        input_tensor = input_tensor.to(device);
    }

    return input_tensor;
}

void TorchModelLoader::ListenerCallback(const InputType &msg){
    latest_message_ = msg;
}

void TorchModelLoader::TimerCallback(){

    if (load_successful_)
    {    
    RCLCPP_INFO(this->get_logger(), 
            "Received input: [%.3f, %.3f, %.3f] at time %d.%09d",
            latest_message_.vector.x, latest_message_.vector.y, latest_message_.vector.z,
            latest_message_.header.stamp.sec, latest_message_.header.stamp.nanosec);
    
    // Pack inputs from the ROS message
    torch::Tensor input_tensor = PackInputs(latest_message_);

    // Get model outputs
    OutputType output_msg;
    GetModelOutputs(input_tensor, output_msg);

    // Log output message
    RCLCPP_INFO(this->get_logger(), 
                "Model output: [%.3f, %.3f, %.3f, %.3f, %.3f]",
                output_msg.model_output[0], output_msg.model_output[1], 
                output_msg.model_output[2], output_msg.model_output[3], 
                output_msg.model_output[4]);

    // Publish the output message
    publisher_->publish(output_msg);
    } else 
    {
        RCLCPP_INFO(this->get_logger(), "Model loading failed");
    }
}
