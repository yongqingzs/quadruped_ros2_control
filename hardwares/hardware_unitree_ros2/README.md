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
