# Network Loader

A ROS 2 package for real-time PyTorch neural network inference.

## Overview

This package provides an example for loading and running PyTorch models within the ROS 2 ecosystem. It accepts 3D vector inputs via ROS messages and publishes model predictions, supporting both CPU and CUDA inference with automatic device detection.

## Installation

1. Update the **path of the LibTorch** in `CMakeLists.txt`
```
list(APPEND CMAKE_PREFIX_PATH "${your_python_path}/site-packages/torch/share/cmake")
find_package(Torch REQUIRED)
```

3. **Build the package**:
```bash
colcon build --packages-select network_loader
source install/setup.bash
```

## Usage

### Quick Start

Launch both publisher and model loader nodes:
```bash
ros2 launch network_loader model_loader_launch.py
```

### Individual Node Execution

Run nodes separately for debugging:
```bash
# Terminal 1 - Data publisher
ros2 run network_loader minimal_publisher_node

# Terminal 2 - Model loader
ros2 run network_loader model_loader_node
```

## Configuration

### Parameters (`config/params.yaml`)

```yaml
model_loader:
  ros__parameters:
    model_name: "array_net_model"           # Base model filename
    model_path: "/path/to/models/"          # Absolute path to model directory
    input_dims: 5                           # Input tensor dimensions
    output_dims: 5                          # Output tensor dimensions
```

### Model Files

Models should be placed in the `models/` directory with device-specific naming:
- `{model_name}_cpu.pt` - CPU inference model
- `{model_name}_cuda.pt` - GPU inference model (if CUDA available)

## Message Interfaces

### Input: `geometry_msgs/Vector3Stamped`
```
std_msgs/Header header
geometry_msgs/Vector3 vector
  float64 x
  float64 y  
  float64 z
```

### Output: `network_loader/Model`
```
std_msgs/Header header
float32[5] model_output
```

## Topics

| Topic Name | Message Type | Description |
|------------|--------------|-------------|
| `/minimal_publisher` | `geometry_msgs/Vector3Stamped` | Input 3D vectors for inference |
| `/model_output_data` | `network_loader/Model` | Model predictions |

## Debug
### `libc10.so` not found errors
If the following error is reported `error while loading shared libraries: libc10.so: cannot open shared object file: No such file or directory`. You need to manually add the torch lib to the environment variable `LD_LIBRARY_PATH`
```bash
export LD_LIBRARY_PATH=/home/orin2/venv/lib/python3.10/site-packages/torch/lib:$LD_LIBRARY_PATH
source ~/.bashrc
```


## Development

### Building from Source

```bash
cd ~/ros_ws
colcon build --packages-select network_loader --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Adding New Models

1. Train your PyTorch model and export using TorchScript:
```python
model = YourModel()
traced_model = torch.jit.trace(model, example_input)
traced_model.save("your_model_cpu.pt")
```

2. Update `config/params.yaml` with new model parameters
3. Place model files in the `models/` directory
4. Rebuild and test



