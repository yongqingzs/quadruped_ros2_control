# Hardware Unitree ROS2

This package provides a ROS2-based hardware interface for Unitree quadruped robots, serving as an alternative to the DDS-based `hardware_unitree_sdk2` package.

## Overview

`hardware_unitree_ros2` implements the `hardware_interface::SystemInterface` from ros2_control and uses standard ROS2 messages from the `unitree_go` package instead of DDS communication.

## Key Features

- **ROS2 Message Integration**: Uses `unitree_go::msg::LowCmd`, `unitree_go::msg::LowState`, and `unitree_go::msg::SportModeState`
- **Hardware Interface Compliance**: Fully compatible with ros2_control framework
- **DDS-ROS2 Bridge Node**: Dedicated bridge for communication with DDS-based hardware/simulation
- **Standard ROS2 Communication**: Primary interface uses standard ROS2 messages
- **Joint Control**: Supports position, velocity, and effort control with PD gains
- **Sensor Data**: Provides IMU, foot force, and high-level state information
- **CRC Processing**: Automatic checksum calculation for DDS communication integrity

## Dependencies

- `hardware_interface`
- `pluginlib` 
- `rclcpp`
- `rclcpp_lifecycle`
- `unitree_go` (ROS2 message definitions)
- `unitree_sdk2` (for DDS bridge functionality)
- `fastrtps`, `fastcdr` (DDS communication libraries)

## Package Structure

```
hardware_unitree_ros2/
├── CMakeLists.txt
├── package.xml
├── include/hardware_unitree_ros2/
│   ├── HardwareUnitreeRos2.h
│   ├── dds_ros2_bridge_node.h
│   └── crc32.h
├── src/
│   ├── HardwareUnitreeRos2.cpp
│   ├── dds_ros2_bridge_node.cpp
│   ├── dds_ros2_bridge_node_main.cpp
│   └── crc32.cpp
├── launch/
│   └── dds_ros2_bridge.launch.py
├── HardwareUnitreeRos2.xml
└── README.md
```

## Usage

This hardware interface is designed to be loaded by the ros2_control framework. It supports two modes:

### Mode 1: Direct ROS2 Communication (Future Hardware)
When the actual hardware directly publishes ROS2 messages:

1. **In your robot description (URDF/XACRO)**:
```xml
<ros2_control name="hardware_unitree_ros2" type="system">
  <hardware>
    <plugin>hardware_unitree_ros2/HardwareUnitreeRos2</plugin>
    <param name="show_foot_force">true</param>
  </hardware>
  <!-- joint and sensor definitions -->
</ros2_control>
```

2. **Load the controller manager**:
```bash
ros2 run controller_manager ros2_control_node --ros-args --params-file your_config.yaml
```

### Mode 2: With DDS Bridge (Current Hardware/Simulation)
When using unitree_mujoco or hardware that communicates via DDS:

1. **Start the DDS-ROS2 bridge**:
```bash
# Using launch file (recommended)
ros2 launch hardware_unitree_ros2 dds_ros2_bridge.launch.py network_interface:=eth0 domain:=1

# Or run directly
ros2 run hardware_unitree_ros2 dds_ros2_bridge_node \
    --ros-args \
    -p network_interface:=eth0 \
    -p domain:=1 \
    -p debug_output:=true
```

2. **Start the hardware interface** (same as Mode 1)

### Bridge Architecture
```
┌─────────────────────┐    DDS     ┌─────────────────────┐    ROS2    ┌─────────────────────┐
│                     │◄──────────►│                     │◄──────────►│                     │
│  unitree_mujoco     │   rt/*     │  dds_ros2_bridge    │ unitree_go/*│ hardware_unitree_   │
│  or Real Hardware   │            │      _node          │             │       ros2          │
│                     │            │                     │             │                     │
└─────────────────────┘            └─────────────────────┘             └─────────────────────┘
```

## Parameters

### Hardware Interface Parameters:
- `show_foot_force` (bool, default: false): Enable/disable foot force logging

### DDS Bridge Node Parameters:
- `network_interface` (string, default: "lo"): Network interface for DDS communication
- `domain` (int, default: 1): DDS domain ID  
- `debug_output` (bool, default: false): Enable debug logging

## Topics

### DDS Topics (used by bridge):
- `rt/lowcmd`: Low-level motor commands (DDS)
- `rt/lowstate`: Low-level robot state (DDS)  
- `rt/sportmodestate`: High-level robot state (DDS)

### ROS2 Topics:
- `unitree_go/low_cmd`: Low-level motor commands (ROS2)
- `unitree_go/low_state`: Low-level robot state (ROS2)
- `unitree_go/high_state`: High-level robot state (ROS2)

## State Interfaces

The hardware interface exports the following state interfaces:

### Joint States (12 joints)
- `position`: Joint position in radians
- `velocity`: Joint velocity in rad/s  
- `effort`: Joint effort/torque in Nm

### IMU Sensor
- `orientation.w, orientation.x, orientation.y, orientation.z`: Quaternion
- `angular_velocity.x, angular_velocity.y, angular_velocity.z`: Gyroscope data
- `linear_acceleration.x, linear_acceleration.y, linear_acceleration.z`: Accelerometer data

### Foot Force Sensors (4 feet)
- `force.z`: Vertical force for each foot

### High-level State (if configured)
- `position.x, position.y, position.z`: Robot base position
- `velocity.x, velocity.y, velocity.z`: Robot base velocity

## Command Interfaces

The hardware interface accepts the following command interfaces:

### Joint Commands (12 joints)
- `position`: Desired joint position
- `velocity`: Desired joint velocity
- `effort`: Desired joint torque
- `kp`: Proportional gain for position control
- `kd`: Derivative gain for velocity control

## Implementation Status

**Current Status**: ✅ **Complete with DDS Bridge**

This implementation provides:
- ✅ Complete hardware interface structure
- ✅ DDS-ROS2 bridge node for communication with existing hardware/simulation
- ✅ CRC checksum calculation for data integrity
- ✅ Full message conversion between DDS and ROS2 formats
- ✅ Thread-safe communication with proper error handling
- ✅ Launch files and parameter configuration

### Task 28 Completion

This package now successfully implements:

1. **DDS Message Reception**: Similar to `hardware_unitree_sdk2`, can receive DDS messages from `unitree_mujoco`
2. **ROS2 Message Translation**: Converts DDS messages to ROS2 messages for `hardware_unitree_ros2`
3. **Bidirectional Bridge**: Handles both DDS→ROS2 and ROS2→DDS conversion
4. **CRC Processing**: Includes CRC32 checksum calculation identical to `hardware_unitree_sdk2`

## Comparison with hardware_unitree_sdk2

| Feature | hardware_unitree_sdk2 | hardware_unitree_ros2 |
|---------|----------------------|----------------------|
| Communication | Direct DDS | ROS2 Messages + DDS Bridge |
| Messages | `unitree_go::msg::dds_::*` | `unitree_go::msg::*` |
| Dependencies | unitree_sdk2 library | Standard ROS2 + Bridge |
| Integration | Direct robot connection | Via bridge node |
| Complexity | Medium (DDS setup) | Lower (standard ROS2) |
| Flexibility | Hardware-specific | Hardware-agnostic |
| CRC Processing | ✅ Built-in | ✅ In bridge node |

## Development Notes

This package was created as Task 28, implementing a DDS-ROS2 bridge that allows `hardware_unitree_ros2` to:
- Communicate with DDS-based hardware/simulation (like `unitree_mujoco`)
- Maintain ROS2-first architecture 
- Provide the same DDS functionality as `hardware_unitree_sdk2`
- Include proper CRC processing for data integrity

The bridge node acts as a translation layer, enabling seamless integration between DDS and ROS2 ecosystems.

## Future Work

- Add complete configuration management
- Implement advanced safety features  
- Add unit tests and integration tests
- Performance optimization for high-frequency control
- Add monitoring and diagnostics

## Troubleshooting

1. **Bridge not receiving DDS messages**: Check network interface and domain settings
2. **CRC errors**: Ensure message structure matches between DDS and ROS2 versions  
3. **Performance issues**: Consider adjusting QoS settings or reducing debug output
4. **Hardware interface timeout**: Verify bridge node is running and topics are connected

## Related Packages

- `hardware_unitree_sdk2`: DDS-based hardware interface
- `unitree_go`: ROS2 message definitions for Unitree robots
- `ros2_control`: Robot control framework

# Data Logging Feature for DDS-ROS2 Bridge

## Overview

This document describes the data logging functionality added to the `dds_ros2_bridge_node` as part of Task 29. The system automatically logs robot state and command data when control input commands change.

## Features

- **Automatic Trigger**: Data logging is triggered when the `command` field in `control_input_msgs::msg::Inputs` changes
- **10-Second Recording**: Records 10 seconds of data after each command change
- **Comprehensive Data**: Logs both `low_state` and `low_cmd` data with timestamps
- **CSV Export**: Automatically saves data to timestamped CSV files
- **Python Visualization**: Included script for plotting and analyzing logged data

## Configuration

### Parameters

- `log_flag` (bool, default: false): Enable/disable data logging functionality
- `network_interface` (string, default: "lo"): DDS network interface
- `domain` (int, default: 1): DDS domain ID
- `debug_output` (bool, default: false): Enable debug logging

### Topics

- **Subscribed**: `control_inputs` (`control_input_msgs::msg::Inputs`)
- **Published**: `unitree_go/low_state`, `unitree_go/high_state`
- **Subscribed**: `unitree_go/low_cmd`

## Usage

### Basic Launch (without logging)

```bash
ros2 launch hardware_unitree_ros2 dds_ros2_bridge.launch.py
```

### Launch with Data Logging

```bash
# Option 1: Using parameter
ros2 launch hardware_unitree_ros2 dds_ros2_bridge.launch.py log_flag:=true

# Option 2: Using dedicated launch file
ros2 launch hardware_unitree_ros2 dds_ros2_bridge_with_logging.launch.py
```

### Publishing Control Commands

To trigger data logging, publish to the control inputs topic:

```bash
# Example: Change command from 0 to 1
ros2 topic pub /control_inputs control_input_msgs/msg/Inputs "{command: 1, lx: 0.0, ly: 0.0, rx: 0.0, ry: 0.0}"

# Change to command 2 (will trigger another 10s recording)
ros2 topic pub /control_inputs control_input_msgs/msg/Inputs "{command: 2, lx: 0.0, ly: 0.0, rx: 0.0, ry: 0.0}"
```

## Data Format

### CSV Structure

The generated CSV files contain the following columns:

#### Metadata
- `timestamp`: Time since logging started (seconds)

#### IMU Data
- `imu_quat_w`, `imu_quat_x`, `imu_quat_y`, `imu_quat_z`: Quaternion orientation
- `imu_gyro_x`, `imu_gyro_y`, `imu_gyro_z`: Angular velocity (rad/s)
- `imu_accel_x`, `imu_accel_y`, `imu_accel_z`: Linear acceleration (m/s²)
- `imu_rpy_r`, `imu_rpy_p`, `imu_rpy_y`: Roll, pitch, yaw angles (rad)
- `imu_temperature`: IMU temperature (°C)

#### Motor State Data (for each of 20 motors)
- `motor_N_mode`: Motor control mode
- `motor_N_q`: Joint position (rad)
- `motor_N_dq`: Joint velocity (rad/s)
- `motor_N_ddq`: Joint acceleration (rad/s²)
- `motor_N_tau`: Estimated torque (Nm)
- `motor_N_w`: Motor angular velocity
- `motor_N_temperature`: Motor temperature (°C)

#### Motor Command Data (for each of 20 motors)
- `cmd_motor_N_mode`: Commanded control mode
- `cmd_motor_N_q`: Commanded position (rad)
- `cmd_motor_N_dq`: Commanded velocity (rad/s)
- `cmd_motor_N_tau`: Commanded torque (Nm)
- `cmd_motor_N_kp`: Position gain
- `cmd_motor_N_kd`: Derivative gain

### File Naming

CSV files are automatically named with timestamps:
```
logged_data_YYYYMMDD_HHMMSS.csv
```

Example: `logged_data_20250129_143052.csv`

## Data Visualization

Use the included Python script to visualize logged data:

```bash
# Basic usage
python3 scripts/visualize_logged_data.py logged_data_20250129_143052.csv

# Save plots as PNG files
python3 scripts/visualize_logged_data.py logged_data_20250129_143052.csv --save

# Focus on specific motor
python3 scripts/visualize_logged_data.py logged_data_20250129_143052.csv --motor-id 5

# Show help
python3 scripts/visualize_logged_data.py --help
```

### Generated Plots

1. **IMU Data**: Quaternion, gyroscope, accelerometer, and RPY plots
2. **Motor Overview**: All motor positions and torques
3. **Detailed Motor Data**: Position, velocity, torque, temperature, and gains for specified motor

### Dependencies

The visualization script requires:
```bash
pip install pandas matplotlib numpy
```

## Implementation Details

### Logging Trigger

- Monitors `control_input_msgs::msg::Inputs.command` field
- Triggers when command value changes from previous value
- Supports any integer command values
- Initial command value of -1 ensures first command always triggers logging

### Data Recording

- Records at the rate of incoming `low_state` messages (~100Hz)
- Stores last 10 seconds of data (up to 1000 samples)
- Uses circular buffer to prevent excessive memory usage
- Thread-safe recording with mutex protection

### File Output

- CSV files written to current working directory
- Automatic timestamp generation for unique filenames
- High precision (6 decimal places) for timestamp data
- Standard CSV format compatible with Excel, MATLAB, Python pandas

## Examples

### Example Launch Sequence

1. Start the bridge with logging:
```bash
ros2 launch hardware_unitree_ros2 dds_ros2_bridge_with_logging.launch.py
```

2. In another terminal, publish a command change:
```bash
ros2 topic pub /control_inputs control_input_msgs/msg/Inputs "{command: 1, lx: 0.5, ly: 0.0, rx: 0.0, ry: 0.0}"
```

3. Wait 10 seconds for logging to complete, then visualize:
```bash
python3 scripts/visualize_logged_data.py logged_data_*.csv --save
```

### Integration with Existing System

This logging system integrates seamlessly with existing hardware_unitree_ros2 operations:

- Does not interfere with normal DDS-ROS2 message bridging
- Minimal performance impact when logging disabled
- Compatible with all existing launch configurations
- Can be enabled/disabled via parameter without code changes

## Troubleshooting

### Common Issues

1. **No CSV files generated**: Ensure `log_flag` is set to `true` and control inputs are being published
2. **Permission errors**: Ensure write permissions in current directory
3. **Visualization errors**: Install required Python packages (pandas, matplotlib, numpy)
4. **Missing motor data**: Check that both low_state and low_cmd topics are active

### Debug Information

Enable debug output to see logging status:
```bash
ros2 launch hardware_unitree_ros2 dds_ros2_bridge_with_logging.launch.py debug_output:=true
```

Look for log messages like:
- "Data logging enabled - subscribed to control inputs"
- "Control command changed from X to Y - starting 10s data logging"
- "Started data logging for 10 seconds"
- "Stopped data logging. Recorded N data points"
- "Saved N data points to filename.csv"

