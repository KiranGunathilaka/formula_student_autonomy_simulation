# Falcon Cone Perception

Real-time cone detection and localization module for the Falcon autonomous racing vehicle.

## Overview

This package provides cone detection capabilities using YOLOv8 deep learning model for camera-based detection and optional pointcloud-based fusion. It detects three types of cones:
- **Yellow Cones** - Track boundaries (right)
- **Blue Cones** - Track boundaries (left)
- **Orange Cones** - Finish line or special markers

## Features

- Real-time YOLOv8-based cone detection
- Multi-class cone detection (Yellow, Blue, Orange)
- Configurable confidence and IOU thresholds
- GPU acceleration support
- ROS 2 integration
- Modular architecture for easy extension

## Package Structure

```
falcon_cone_perception/
├── launch/
│   └── cone_detection.launch.py    # ROS 2 launch file
├── config/
│   ├── yolo_detector.yaml          # YOLO model configuration
│   └── topic_remaps.yaml           # Topic remapping configuration
├── models/
│   ├── best.pt                     # Pre-trained YOLOv8 model
│   └── README.md                   # Model documentation
├── src/
│   ├── cone_perception_node.cpp    # C++ perception node (if applicable)
│   └── (Python modules here)
├── scripts/
│   └── (Helper scripts and adaptors)
├── package.xml                     # ROS 2 package manifest
├── setup.py                        # Python package setup
├── CMakeLists.txt                  # CMake build configuration
└── README.md                       # This file
```

## Dependencies

### ROS 2 Dependencies
- `rclcpp` / `rclpy` - ROS 2 client library
- `std_msgs` - Standard ROS message types
- `geometry_msgs` - Geometry messages
- `sensor_msgs` - Sensor message types
- `falcon_msgs` - Custom Falcon messages

### External Dependencies
- `ultralytics` - YOLOv8 framework
- `opencv-python` - Image processing
- `torch` / `pytorch` - Deep learning framework
- `numpy` - Numerical computing

## Installation

1. Clone the repository:
```bash
cd ~/falcon_ws/src
# Repository already cloned
```

2. Install dependencies:
```bash
cd ~/falcon_ws
rosdep install --from-paths src --ignore-src -r -y
pip install ultralytics opencv-python torch
```

3. Build the workspace:
```bash
colcon build --packages-select falcon_cone_perception
source install/setup.bash
```

## Usage

### Launch the Cone Detection Node

```bash
ros2 launch falcon_cone_perception cone_detection.launch.py
```

### Configuration

Edit configuration files before launching:
- **yolo_detector.yaml** - Model path, thresholds, class definitions
- **topic_remaps.yaml** - Input/output topic names

### ROS 2 Topics

#### Subscribed Topics
- `/camera/image_raw` (sensor_msgs/Image) - Camera input

#### Published Topics
- `/detected_cones` (falcon_msgs/ConeDetections) - Detected cones

## Development

### Adding Custom Detection Algorithms

1. Create a new module in `src/`
2. Implement the detection interface
3. Register in the main perception node
4. Add configuration to `config/`

### Training Custom Models

See `models/README.md` for instructions on training YOLOv8 models with custom data.

## Performance Metrics

- **Inference Speed**: ~30ms per frame on GPU
- **Accuracy**: [Add metrics after training]
- **Resolution**: 640x480

## Troubleshooting

### Model Not Found
- Check the `model_path` in `yolo_detector.yaml`
- Ensure `models/best.pt` is present and accessible

### No GPU Available
- Set `use_gpu: false` in configuration
- Ensure CUDA and cuDNN are properly installed for GPU support

### Detection Quality Issues
- Adjust `confidence_threshold` in configuration
- Ensure good lighting conditions
- Check camera calibration

## Future Enhancements

- [ ] Pointcloud-based cone fusion
- [ ] 3D cone localization
- [ ] Multi-camera support
- [ ] Real-time model optimization
- [ ] Cone tracking across frames

## Contributing

Contributions are welcome! Please follow the coding standards and submit pull requests to the team.

## License

MIT License - See LICENSE file in the repository root

## Authors

FalconE Racing Team

## References

- [YOLOv8 Documentation](https://github.com/ultralytics/ultralytics)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
