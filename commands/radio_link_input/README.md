# Radio Link Input Package

This ROS2 package provides a node for processing SBUS (Serial Bus) radio controller input for quadruped robots.

## Features

- Reads SBUS protocol data from serial port
- Parses 16 SBUS channels
- Publishes `sensor_msgs/Joy` messages for joystick input
- Publishes `control_input_msgs/Inputs` messages for robot control
- Implements state machine based on mode switch position
- Configurable channel mappings via ROS2 parameters

## SBUS Protocol

SBUS is a serial protocol used by FrSky and other radio control systems. It provides 16 channels with 11-bit resolution (0-2047 range, typically 172-1811 used).

## Robot States

The mode channel controls the robot state machine:

- **STOP**: Emergency stop
- **STAND_DOWN**: Lower robot to ground
- **STAND_UP**: Raise robot to standing position
- **WALK**: Normal walking mode
- **RUN**: High-speed running mode

## Parameters

- `serial_port` (string, default: "/dev/ttyUSB0"): Serial port device
- `baud_rate` (int, default: 100000): SBUS baud rate (fixed at 100k)
- `lx_channel` (int, default: 0): Left stick X-axis channel
- `ly_channel` (int, default: 1): Left stick Y-axis channel
- `rx_channel` (int, default: 2): Right stick X-axis channel
- `ry_channel` (int, default: 3): Right stick Y-axis channel
- `mode_channel` (int, default: 4): Mode switch channel

## Topics

### Published Topics

- `joy` (`sensor_msgs/Joy`): Raw joystick data with all 16 SBUS channels as axes and state buttons
- `inputs` (`control_input_msgs/Inputs`): Processed control inputs with command and normalized axes

### Message Formats

#### control_input_msgs/Inputs
```
int32 command    # Robot state command (0=STOP, 1=STAND_UP, 2=STAND_DOWN, 3=WALK, 4=RUN)
float32 lx       # Left X-axis (-1.0 to 1.0)
float32 ly       # Left Y-axis (-1.0 to 1.0)
float32 rx       # Right X-axis (-1.0 to 1.0)
float32 ry       # Right Y-axis (-1.0 to 1.0)
```

## Usage

### Launch the node

```bash
ros2 launch radio_link_input radio_link_input.launch.xml
```

### Custom parameters

```bash
ros2 launch radio_link_input radio_link_input.launch.xml serial_port:=/dev/ttyACM0 lx_channel:=1 ly_channel:=2
```

### Check topics

```bash
ros2 topic list
ros2 topic echo /inputs
ros2 topic echo /joy
```

## Hardware Setup

1. Connect SBUS receiver to serial port (typically `/dev/ttyUSB0` or `/dev/ttyACM0`)
2. Configure radio transmitter channels according to your mapping
3. Ensure proper permissions for serial port access

## Dependencies

- ROS2 (rclcpp)
- sensor_msgs
- control_input_msgs (custom message package)

## Building

```bash
cd ~/jazzy_ws
colcon build --packages-select radio_link_input
```

## Troubleshooting

- **Serial port not found**: Check device connection and permissions
- **No data received**: Verify SBUS receiver connection and transmitter binding
- **Wrong channel values**: Check channel mappings and transmitter configuration
- **Failsafe active**: Check radio link quality and receiver status

## rc_controller_startup

### 设置 systemd 服务（系统启动时运行脚本）
创建服务文件 `/etc/systemd/system/rc-controller.service`：

```ini
[Unit]
Description=RC Controller Startup Service
After=network.target

[Service]
Type=simple
User=cat  # 替换为你的用户名
ExecStart=/usr/bin/python3 /home/cat/rc_controller_startup.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

- 启用并启动服务：
  ```bash
  sudo systemctl daemon-reload
  sudo systemctl enable rc-controller.service
  sudo systemctl start rc-controller.service
  ```
- 检查状态：`sudo systemctl status rc-controller.service`
- 查看日志：`journalctl -u rc-controller.service -f`
