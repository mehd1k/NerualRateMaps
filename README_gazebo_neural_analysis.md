# Gazebo Neural Analysis Node

## Overview
This ROS node integrates Gazebo camera data with your MATLAB neural analysis pipeline, replacing the original Panda3D implementation while maintaining the same image processing and neural analysis workflow.

## Features
- **Gazebo Integration**: Subscribes to Gazebo camera topics for real-time image processing
- **MATLAB Integration**: Uses MATLAB engine for neural rate analysis
- **Control System**: Implements the same control logic as the original Panda3D version
- **Data Logging**: Saves trajectory, neural rates, and control inputs
- **Visualization**: Creates trajectory plots and debug images

## Requirements

### System Requirements
- ROS 2 (Humble/Iron recommended)
- Python 3.8+
- MATLAB with Python API
- OpenCV
- NumPy, Matplotlib
- Gazebo simulation environment

### Dependencies
```bash
# ROS 2 packages
sudo apt install ros-humble-cv-bridge ros-humble-message-filters ros-humble-tf2-ros

# Python packages
pip install opencv-python numpy matplotlib scipy
pip install matlab
```

## Installation

1. **Clone the repository**:
```bash
cd ~/NerualRateMaps
```

2. **Make the script executable**:
```bash
chmod +x gazebo_neural_analysis.py
```

3. **Ensure MATLAB is properly configured**:
```bash
# Test MATLAB Python API
python -c "import matlab.engine; print('MATLAB API available')"
```

## Usage

### Basic Usage
```bash
# Run the node directly
ros2 run neural_rate_maps gazebo_neural_analysis.py
```

### Using Launch File
```bash
# Launch with default parameters
ros2 launch neural_rate_maps gazebo_neural_analysis.launch.py

# Launch with custom parameters
ros2 launch neural_rate_maps gazebo_neural_analysis.launch.py \
    camera_topic:=/my_camera/image_raw \
    num_steps:=200 \
    initial_x:=0.5 \
    initial_y:=0.3
```

### Launch Parameters
- `camera_topic`: Gazebo camera topic (default: `/my_camera/image_raw`)
- `num_steps`: Number of processing steps (default: 150)
- `initial_x`: Initial X position (default: 0.4)
- `initial_y`: Initial Y position (default: 0.25)
- `initial_hd`: Initial heading direction (default: 0.0)
- `processing_rate`: Processing rate in Hz (default: 10.0)
- `output_dir`: Output directory (default: ./output)
- `use_rviz`: Launch RViz for visualization (default: false)

## Configuration

### Camera Setup in Gazebo
Ensure your Gazebo simulation publishes camera data on the configured topic:
```xml
<!-- Example camera plugin in Gazebo SDF -->
<plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
    <ros>
        <namespace>/my_camera</namespace>
        <remapping>image_raw:=image_raw</remapping>
    </ros>
    <camera_name>camera</camera_name>
    <image_topic>image_raw</image_topic>
    <camera_info_topic>camera_info</camera_info_topic>
    <frame_name>camera_link</frame_name>
</plugin>
```

### MATLAB Function
The node expects a MATLAB function `generate_V1_RSC_model_response` that takes:
- `image`: Input image matrix
- `S`: Initial state vector (100x1)
- `U`: Initial control vector (100x1)

And returns:
- `S_out`: Neural rate output (100x1)
- `U_out`: Updated control vector (100x1)

## Output Data

### Generated Files
The node creates the following output files in the `trj/` directory:

- `postion_ls.npy`: Robot positions over time
- `hd_ls.npy`: Heading directions over time
- `u_ls.npy`: Control inputs over time
- `image_ls.npy`: Processed images over time
- `ratemap_ls.npy`: Neural rate maps over time
- `trajectory_plot.png`: Trajectory visualization

### Debug Images
Debug images are saved in `test/img/` with naming pattern:
```
step_XXX_pos_X.XX_Y.XX_hd_XXX.X.png
```

## Troubleshooting

### Common Issues

1. **MATLAB Engine Not Starting**:
   ```bash
   # Check MATLAB installation
   matlab -batch "disp('MATLAB working')"
   
   # Test Python API
   python -c "import matlab.engine; eng = matlab.engine.start_matlab(); eng.quit()"
   ```

2. **Camera Topic Not Found**:
   ```bash
   # List available topics
   ros2 topic list | grep camera
   
   # Check camera data
   ros2 topic echo /my_camera/image_raw --once
   ```

3. **Cell Control Gains Missing**:
   - Ensure `cells_controllers/` directory exists
   - Verify control gain files (`K.npy`, `Kb.npy`) are present
   - Check cell definitions in `gen_controller.py`

### Performance Optimization

1. **Reduce Processing Rate**: Lower `processing_rate` parameter for slower systems
2. **Disable Debug Images**: Set `debug_images: false` in config
3. **MATLAB Engine**: Consider using MATLAB Compiler for better performance

## Integration with Original Code

This node maintains compatibility with your existing codebase:

- **Image Format**: Same 150x90 grayscale format as Panda3D version
- **Neural Analysis**: Uses identical MATLAB function calls
- **Control System**: Same control logic and cell-based navigation
- **Data Format**: Compatible output format for analysis

## Customization

### Adding New Cell Types
1. Define new cells in `gen_controller.py`
2. Generate control gains using `gen_controller_all_orinetation()`
3. Update `cell_ls` in the node

### Modifying Control Logic
Edit the `controller()` and `update_state()` methods in `gazebo_neural_analysis.py`

### Custom Image Processing
Modify the `process_image()` method to implement custom preprocessing

## Support

For issues or questions:
1. Check the logs: `ros2 log gazebo_neural_analysis`
2. Verify MATLAB function works independently
3. Test with simple Gazebo simulation first
4. Check file permissions for output directories
