# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 package for loading and running PyTorch neural network models within the ROS ecosystem. The package provides real-time inference capabilities for neural networks, accepting Vector3Stamped messages as input and publishing model predictions as custom Model messages.

## Build and Development Commands

### Building the Package
```bash
# From the ROS workspace root (likely ~/ros_ws or similar)
colcon build --packages-select network_loader
source install/setup.bash
```

### Running the Package
```bash
# Launch both publisher and model loader nodes
ros2 launch network_loader model_loader_launch.py

# Run individual nodes
ros2 run network_loader minimal_publisher_node
ros2 run network_loader model_loader_node

# With custom launch arguments (objects to track)
ros2 launch network_loader model_loader_launch.py objects:="x500_0 x500_1"
```

### Testing and Linting
```bash
# Run ROS 2 tests (if available)
colcon test --packages-select network_loader

# Run linting checks
ament_cpplint src/
ament_cppcheck src/
```

## Architecture

### Build System Structure
- **CMakeLists.txt**: Root build configuration with PyTorch integration
  - Uses conda environment at `~/miniconda3/envs/py39/` for LibTorch
  - Configures C++17 standard and warning flags
  - Manages custom message generation and dependencies
- **src/CMakeLists.txt**: Executable definitions linking to model_loader_lib
- **src/lib/CMakeLists.txt**: Static library build with PyTorch and Eigen3

### Core Components

1. **TorchModelLoader** (`src/lib/torch_model_loader.cpp/.hpp`): Main inference node
   - Subscribes to `minimal_publisher` topic (Vector3Stamped messages)
   - Publishes to `model_output_data` topic (custom Model messages)
   - Device-aware model loading (CPU/CUDA automatic detection)
   - Parameter-driven configuration from `config/params.yaml`

2. **MinimalPublisher** (`src/minimimal_publisher_node.cpp`): Test data publisher
   - Publishes Vector3Stamped messages for testing the inference pipeline

3. **Model Message** (`msg/Model.msg`): Custom ROS 2 interface
   - Contains `std_msgs/Header header` for timestamp tracking
   - Includes `float32[5] model_output` array for predictions

4. **Launch System** (`launch/model_loader_launch.py`): 
   - Configurable object tracking via launch arguments
   - Absolute path resolution for parameter files
   - Dual-node orchestration (publisher + model loader)

### Model Management
- Pre-trained PyTorch models stored in `models/` directory (not `python/`)
- Device-specific models: `*_cpu.pt` and `*_cuda.pt` variants
- JIT-compiled models loaded via `torch::jit::load()`
- Input: 3D vectors → Output: 5-element float arrays

### Configuration System
- **config/params.yaml**: Centralized parameter management
  - `model_name`: Base name for model files (without device suffix)
  - `model_path`: Absolute path to model directory
  - `input_dims`/`output_dims`: Model tensor dimensions (currently 5/5)
- Node name must match launch file configuration (`model_loader`)

### Dependencies and Build Requirements
- **ROS 2**: rclcpp, std_msgs, geometry_msgs, nav_msgs, rosidl_default_generators
- **PyTorch**: LibTorch C++ API with conda environment integration
- **Linear Algebra**: Eigen3 for tensor operations
- **Build Tools**: ament_cmake with custom message generation support

## Key Implementation Patterns

### Model Loading Workflow
1. Parameter validation and device detection
2. Model file path construction based on device type
3. PyTorch module loading and device transfer
4. Subscription/publication setup with QoS configuration

### Message Processing Pipeline
- **Input**: geometry_msgs::Vector3Stamped → tensor conversion
- **Inference**: Device-aware PyTorch forward pass
- **Output**: Tensor extraction → network_loader::Model message publication

### Error Handling Strategy
- `load_successful_` flag tracks model loading 
- `has_new_message_` flag tracks recieved message
- Device fallback from CUDA to CPU when appropriate
- Parameter validation with default value fallbacks
