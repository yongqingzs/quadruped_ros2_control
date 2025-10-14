#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from control_input_msgs.msg import Inputs
import serial
import threading
import time
import math
import subprocess
import os

class RadioLinkInput(Node):
    def __init__(self):
        super().__init__('radio_link_input')

        # Declare parameters (matching C++ defaults)
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 100000)
        self.declare_parameter('lx_channel', 3)
        self.declare_parameter('ly_channel', 2)
        self.declare_parameter('rx_channel', 0)
        self.declare_parameter('ry_channel', 1)
        self.declare_parameter('mode_0', 9)
        self.declare_parameter('mode_1', 6)
        self.declare_parameter('mode_2', 4)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.channel_mapping = {
            'lx_channel': self.get_parameter('lx_channel').get_parameter_value().integer_value,
            'ly_channel': self.get_parameter('ly_channel').get_parameter_value().integer_value,
            'rx_channel': self.get_parameter('rx_channel').get_parameter_value().integer_value,
            'ry_channel': self.get_parameter('ry_channel').get_parameter_value().integer_value,
            'mode_0': self.get_parameter('mode_0').get_parameter_value().integer_value,
            'mode_1': self.get_parameter('mode_1').get_parameter_value().integer_value,
            'mode_2': self.get_parameter('mode_2').get_parameter_value().integer_value,
        }

        # Create publisher
        self.inputs_publisher = self.create_publisher(Inputs, 'control_input', 10)

        # Serial communication
        self.serial_fd = None
        self.running = False
        self.serial_thread = None
        self.data_mutex = threading.Lock()

        # SBUS data
        self.SBUS_FRAME_SIZE = 25
        self.SBUS_NUM_CHANNELS = 16
        self.channels = [0] * self.SBUS_NUM_CHANNELS
        self.failsafe = False
        self.frame_lost = False

        # State machine
        self.RobotState = {
            'RUN': 0,
            'STOP': 1,
            'STAND': 2,
            'TROT': 3,
        }
        self.current_state = self.RobotState['STOP']

        # State tracking for determineRobotState
        self.last_normal_0 = 0.0
        self.last_normal_1 = 0.0
        self.last_state = self.RobotState['STOP']

        # Controller management (integrated from rc_controller_startup.py)
        self.controller_process = None
        self.controller_running = False
        self.last_ch4_normal = 1.0

        # ROS2 environment settings
        self.ROS_SETUP = '/opt/ros/jazzy/setup.bash'
        self.WORKSPACE_SETUP = '/home/cat/jazzy_ws/install/setup.bash'
        self.LAUNCH_FILE = '/home/cat/jazzy_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/launch/mujoco.launch.py'

        # Setup serial
        if self.setup_serial():
            self.running = True
            self.serial_thread = threading.Thread(target=self.serial_read_loop)
            self.serial_thread.start()
            self.get_logger().info('RadioLinkInput node started successfully')
        else:
            self.get_logger().error('Failed to setup serial communication')

    def __del__(self):
        self.running = False
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join()
        if self.serial_fd:
            self.serial_fd.close()
        # Stop controller on destruction
        self.stop_controller()

    def setup_serial(self):
        try:
            # Use pyserial for serial communication
            self.serial_fd = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_TWO,
                timeout=0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.serial_fd.flushInput()
            self.serial_fd.flushOutput()
            self.get_logger().info(f'Serial port {self.serial_port} configured successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Error opening serial port: {e}')
            return False

    def serial_read_loop(self):
        buffer = bytearray()
        while self.running:
            try:
                byte = self.serial_fd.read(1)
                if byte:
                    byte = byte[0]
                    # Look for start byte (0x0F)
                    if not buffer and byte == 0x0F:
                        buffer.append(byte)
                    elif buffer:
                        buffer.append(byte)
                        # Check if we have a complete frame
                        if len(buffer) >= self.SBUS_FRAME_SIZE:
                            # Process the frame
                            with self.data_mutex:
                                if self.parse_sbus_frame(buffer):
                                    self.process_channels()
                                    self.publish_inputs_message()
                            buffer.clear()
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
                time.sleep(0.01)
            time.sleep(0.0001)  # Small delay

    def parse_sbus_frame(self, data):
        # SBUS frame format validation
        if len(data) < self.SBUS_FRAME_SIZE or data[0] != 0x0F or data[24] != 0x00:
            return False

        # Extract channel data (11 bits per channel, 16 channels)
        self.channels[0]  = ((data[1]     | data[2] << 8)                  & 0x07FF)
        self.channels[1]  = ((data[2]>>3  | data[3] << 5)                  & 0x07FF)
        self.channels[2]  = ((data[3]>>6  | data[4] << 2 | data[5] << 10)  & 0x07FF)
        self.channels[3]  = ((data[5]>>1  | data[6] << 7)                  & 0x07FF)
        self.channels[4]  = ((data[6]>>4  | data[7] << 4)                  & 0x07FF)
        self.channels[5]  = ((data[7]>>7  | data[8] << 1 | data[9] << 9)   & 0x07FF)
        self.channels[6]  = ((data[9]>>2  | data[10] << 6)                 & 0x07FF)
        self.channels[7]  = ((data[10]>>5 | data[11] << 3)                 & 0x07FF)
        self.channels[8]  = ((data[12]    | data[13] << 8)                 & 0x07FF)
        self.channels[9]  = ((data[13]>>3 | data[14] << 5)                 & 0x07FF)
        self.channels[10] = ((data[14]>>6 | data[15] << 2 | data[16] << 10) & 0x07FF)
        self.channels[11] = ((data[16]>>1 | data[17] << 7)                 & 0x07FF)
        self.channels[12] = ((data[17]>>4 | data[18] << 4)                 & 0x07FF)
        self.channels[13] = ((data[18]>>7 | data[19] << 1 | data[20] << 9) & 0x07FF)
        self.channels[14] = ((data[20]>>2 | data[21] << 6)                 & 0x07FF)
        self.channels[15] = ((data[21]>>5 | data[22] << 3)                 & 0x07FF)

        # Extract flags
        flags = data[23]
        self.failsafe = (flags & 0x08) != 0
        self.frame_lost = (flags & 0x04) != 0

        return True

    def process_channels(self):
        # Determine robot state from mode channel
        mode_0 = self.channels[self.channel_mapping['mode_0']]
        mode_1 = self.channels[self.channel_mapping['mode_1']]
        mode_2 = self.channels[self.channel_mapping['mode_2']]
        self.current_state = self.determine_robot_state(mode_0, mode_1, mode_2)

        # Update state tracking
        self.last_normal_0 = self.normalize_channel_value(mode_0)
        self.last_normal_1 = self.normalize_channel_value(mode_1)
        if self.current_state != self.RobotState['RUN']:
            self.last_state = self.current_state

        # Controller management based on channel 4 (integrated from rc_controller_startup.py)
        ch4 = self.channels[4]
        ch4_normal = self.normalize_channel_value(ch4)
        delta = ch4_normal - self.last_ch4_normal
        if delta > 0.5 and ch4_normal > 0.5:
            self.get_logger().info(f"Starting controller with ch4_normal: {ch4_normal}")
            self.start_controller()
        elif delta < -0.5 and ch4_normal < -0.5:
            self.get_logger().info(f"Stopping controller with ch4_normal: {ch4_normal}")
            self.stop_controller()
        self.last_ch4_normal = ch4_normal

    def publish_inputs_message(self):
        inputs_msg = Inputs()
        inputs_msg.command = self.current_state

        # Map channels to control inputs
        if inputs_msg.command == 0:
            inputs_msg.lx = self.normalize_channel_value(self.channels[self.channel_mapping['lx_channel']])
            inputs_msg.ly = self.normalize_channel_value(self.channels[self.channel_mapping['ly_channel']])
            inputs_msg.rx = self.normalize_channel_value(self.channels[self.channel_mapping['rx_channel']])
            inputs_msg.ry = self.normalize_channel_value(self.channels[self.channel_mapping['ry_channel']])

        self.inputs_publisher.publish(inputs_msg)

    def normalize_channel_value(self, value):
        clamped = (value - 307.0) / (1693.0 - 307.0) * 2.0 - 1.0
        if abs(clamped) < 0.05:
            clamped = 0.0  # Deadzone
        return clamped

    def determine_robot_state(self, mode_0, mode_1, mode_2):
        normal_0 = self.normalize_channel_value(mode_0)
        normal_1 = self.normalize_channel_value(mode_1)
        normal_2 = self.normalize_channel_value(mode_2)

        # Calculate changes
        delta_0 = normal_0 - self.last_normal_0
        delta_1 = normal_1 - self.last_normal_1
        update_state = self.RobotState['RUN']

        def safe_update(target):
            nonlocal update_state
            if target != self.last_state:
                update_state = target

        if delta_0 > 0.5:
            safe_update(self.RobotState['STAND'])
        elif delta_0 < -0.5:
            safe_update(self.RobotState['STOP'])
        elif delta_1 > 1.0 and self.last_state == self.RobotState['STAND']:
            safe_update(self.RobotState['TROT'])
        elif delta_1 < -1.0 and self.last_state == self.RobotState['TROT']:
            safe_update(self.RobotState['STAND'])

        return update_state

    def start_controller(self):
        if self.controller_running:
            self.get_logger().info("Controller already running")
            return
        try:
            # Source ROS2 environment and launch
            cmd = f"source {self.ROS_SETUP} && source {self.WORKSPACE_SETUP} && ros2 launch {self.LAUNCH_FILE}"
            self.controller_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash')
            self.controller_running = True
            self.get_logger().info("Controller started")
        except Exception as e:
            self.get_logger().error(f"Failed to start controller: {e}")

    def stop_controller(self):
        if not self.controller_running:
            self.get_logger().info("Controller not running")
            return
        try:
            if self.controller_process:
                self.controller_process.terminate()
                self.controller_process.wait(timeout=10)
            self.controller_running = False
            self.get_logger().info("Controller stopped")
            # Kill any remaining rviz processes
            result = subprocess.run(["pkill", "-f", "rviz"], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("Rviz processes killed successfully")
            else:
                self.get_logger().warning(f"Failed to kill rviz processes: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"Failed to stop controller: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RadioLinkInput()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()