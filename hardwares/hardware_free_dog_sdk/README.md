# Hardware Free Dog SDK

这是一个基于 Free Dog SDK 的 ROS2 Control 硬件接口包，用于控制基于 Free Dog SDK 的四足机器人。

## 功能特性

- 集成 Free Dog SDK 进行低级硬件通信
- 支持关节位置、速度和力矩控制
- 提供 IMU 传感器数据
- 支持足端力传感器数据
- 可选的高级状态数据支持
- 线程安全的数据通信

## 依赖项

- `fdsc_utils`: Free Dog SDK 工具库
- `hardware_interface`: ROS2 Control 硬件接口
- `rclcpp`: ROS2 C++ 客户端库
- `rclcpp_lifecycle`: ROS2 生命周期管理
- `unitree_go`: Unitree 消息定义 (用于兼容性)

## 参数配置

### 硬件参数

- `connection_settings`: Free Dog SDK 连接设置 (默认: "LOW_WIRED_DEFAULTS")
- `show_foot_force`: 是否在日志中显示足端力数据 (默认: false)
- `use_high_level`: 是否启用高级状态数据 (默认: false)
- `power_limit`: 功率限制设置 (可选)

### 接口定义

#### 状态接口 (State Interfaces)

**关节状态:**
- `position`: 关节位置 (弧度)
- `velocity`: 关节速度 (弧度/秒)
- `effort`: 关节力矩 (牛顿·米)

**IMU 传感器:**
- `orientation.w`, `orientation.x`, `orientation.y`, `orientation.z`: 四元数方向
- `angular_velocity.x`, `angular_velocity.y`, `angular_velocity.z`: 角速度
- `linear_acceleration.x`, `linear_acceleration.y`, `linear_acceleration.z`: 线加速度

**足端力传感器:**
- `force.rf`, `force.lf`, `force.rh`, `force.lh`: 四个足端的垂直力

**高级状态 (可选):**
- `position.x`, `position.y`, `position.z`: 机体位置
- `velocity.x`, `velocity.y`, `velocity.z`: 机体速度

#### 命令接口 (Command Interfaces)

**关节命令:**
- `position`: 目标关节位置
- `velocity`: 目标关节速度
- `effort`: 目标关节力矩
- `kp`: 位置增益
- `kd`: 阻尼增益

## 使用方法

### 基本启动

```bash
ros2 launch hardware_free_dog_sdk visualize.launch.py
```

### 自定义参数启动

```bash
ros2 launch hardware_free_dog_sdk visualize.launch.py \\
  connection_settings:="HIGH_WIFI_DEFAULTS" \\
  show_foot_force:=true \\
  use_high_level:=true
```

### 在 ROS2 Control 配置中使用

```xml
<ros2_control name="free_dog_hardware" type="system">
  <hardware>
    <plugin>HardwareFreeDogSdk</plugin>
    <param name="connection_settings">LOW_WIRED_DEFAULTS</param>
    <param name="show_foot_force">false</param>
    <param name="use_high_level">false</param>
    <param name="power_limit">200.0</param>
  </hardware>
  
  <!-- 关节定义 -->
  <joint name="joint_name">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <command_interface name="kp"/>
    <command_interface name="kd"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <!-- 传感器定义 -->
  <sensor name="imu">
    <state_interface name="orientation.w"/>
    <state_interface name="orientation.x"/>
    <state_interface name="orientation.y"/>
    <state_interface name="orientation.z"/>
    <state_interface name="angular_velocity.x"/>
    <state_interface name="angular_velocity.y"/>
    <state_interface name="angular_velocity.z"/>
    <state_interface name="linear_acceleration.x"/>
    <state_interface name="linear_acceleration.y"/>
    <state_interface name="linear_acceleration.z"/>
  </sensor>
</ros2_control>
```

## 架构设计

### 与 hardware_unitree_sdk2 的对比

| 特性 | hardware_unitree_sdk2 | hardware_free_dog_sdk |
|------|----------------------|----------------------|
| 通信协议 | DDS (unitree_sdk2) | UDP (free_dog_sdk_cpp) |
| 数据格式 | Unitree DDS 消息 | Free Dog SDK 自定义格式 |
| 线程模型 | DDS 回调驱动 | 独立通信线程 |
| 同步机制 | DDS 内置同步 | Mutex 保护 |

### 数据流

```
Free Dog Robot <--UDP--> FDSC::UnitreeConnection <---> HardwareFreeDogSdk <---> ROS2 Control
```

### 线程安全

- 使用 `std::mutex` 保护共享数据
- 独立的通信线程处理网络 I/O
- 原子变量控制线程生命周期

## 参考实现

本包的实现参考了以下代码:

1. **hardware_unitree_sdk2**: ROS2 Control 接口设计和结构
2. **legged_unitree_hw_free**: Free Dog SDK 的集成方式和数据转换

## 注意事项

1. **网络配置**: 确保机器人和控制计算机在同一网络段
2. **权限要求**: 可能需要 sudo 权限进行网络通信
3. **通信频率**: 默认 1kHz 通信频率，可根据需要调整
4. **错误处理**: 包含基本的错误检测和恢复机制

## 故障排除

### 常见问题

1. **连接失败**: 检查网络配置和 connection_settings 参数
2. **数据不更新**: 确认机器人状态和网络连接
3. **编译错误**: 确保所有依赖项正确安装

### 调试信息

启用详细日志:
```bash
ros2 launch hardware_free_dog_sdk visualize.launch.py --ros-args --log-level debug
```

## 开发和扩展

### 添加新的传感器接口

1. 在头文件中添加数据成员
2. 在 `export_state_interfaces()` 中导出接口
3. 在 `read()` 方法中更新数据

### 修改通信参数

编辑构造函数中的 Free Dog SDK 初始化参数，或通过硬件参数传递。

## 许可证

Apache-2.0 License