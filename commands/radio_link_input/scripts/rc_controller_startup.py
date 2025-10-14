#!/usr/bin/env python3
import serial
import time
import subprocess
import os
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# ROS2 环境设置（假设 Jazzy）
ROS_SETUP = '/opt/ros/jazzy/setup.bash'
WORKSPACE_SETUP = '/home/cat/jazzy_ws/install/setup.bash'


# 串口配置（与 radio_link_input.py 保持一致）
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 100000
SBUS_FRAME_SIZE = 25
SBUS_NUM_CHANNELS = 16

# 通道映射（与 radio_link_input.py 保持一致）
CHANNEL_MAPPING = {
    'lx_channel': 3,
    'ly_channel': 2,
    'rx_channel': 0,
    'ry_channel': 1,
    'mode_0': 9,
    'mode_1': 6,
    'mode_2': 4,
}

# Launch 文件路径
LAUNCH_FILE = '/home/cat/jazzy_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/launch/mujoco.launch.py'

class RCController:
    def __init__(self):
        self.serial_conn = None
        self.controller_process = None
        self.running = False

    def setup_serial(self):
        try:
            self.serial_conn = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_TWO,
                timeout=0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            logging.info(f"Serial port {SERIAL_PORT} opened successfully")
        except Exception as e:
            logging.error(f"Failed to open serial port: {e}")
            return False
        return True

    def parse_sbus(self, data):
        # SBUS 解析，返回 channels 数组（与 radio_link_input.py 保持一致）
        if len(data) < SBUS_FRAME_SIZE or data[0] != 0x0F or data[24] != 0x00:
            return None
        channels = [0] * SBUS_NUM_CHANNELS
        channels[0]  = ((data[1]     | data[2] << 8)                  & 0x07FF)
        channels[1]  = ((data[2]>>3  | data[3] << 5)                  & 0x07FF)
        channels[2]  = ((data[3]>>6  | data[4] << 2 | data[5] << 10)  & 0x07FF)
        channels[3]  = ((data[5]>>1  | data[6] << 7)                  & 0x07FF)
        channels[4]  = ((data[6]>>4  | data[7] << 4)                  & 0x07FF)
        channels[5]  = ((data[7]>>7  | data[8] << 1 | data[9] << 9)   & 0x07FF)
        channels[6]  = ((data[9]>>2  | data[10] << 6)                 & 0x07FF)
        channels[7]  = ((data[10]>>5 | data[11] << 3)                 & 0x07FF)
        channels[8]  = ((data[12]    | data[13] << 8)                 & 0x07FF)
        channels[9]  = ((data[13]>>3 | data[14] << 5)                 & 0x07FF)
        channels[10] = ((data[14]>>6 | data[15] << 2 | data[16] << 10) & 0x07FF)
        channels[11] = ((data[16]>>1 | data[17] << 7)                 & 0x07FF)
        channels[12] = ((data[17]>>4 | data[18] << 4)                 & 0x07FF)
        channels[13] = ((data[18]>>7 | data[19] << 1 | data[20] << 9) & 0x07FF)
        channels[14] = ((data[20]>>2 | data[21] << 6)                 & 0x07FF)
        channels[15] = ((data[21]>>5 | data[22] << 3)                 & 0x07FF)
        return channels

    def start_controller(self):
        if self.running:
            logging.info("Controller already running")
            return
        try:
            # Source ROS2 环境并启动 launch
            cmd = f"source {ROS_SETUP} && source {WORKSPACE_SETUP} && ros2 launch {LAUNCH_FILE}"
            self.controller_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash')
            self.running = True
            logging.info("Controller started")
        except Exception as e:
            logging.error(f"Failed to start controller: {e}")

    def stop_controller(self):
        if not self.running:
            logging.info("Controller not running")
            return
        try:
            if self.controller_process:
                self.controller_process.terminate()
                self.controller_process.wait(timeout=10)
            self.running = False
            logging.info("Controller stopped")
            logging.info("Killing any remaining rviz processes...")
            result = subprocess.run(["pkill", "-f", "rviz"], capture_output=True, text=True)
            if result.returncode == 0:
                logging.info("Rviz processes killed successfully")
            else:
                logging.warning(f"Failed to kill rviz processes: {result.stderr}")
        except Exception as e:
            logging.error(f"Failed to stop controller: {e}")

    def normalize_channel_value(self, value):
        # 与 radio_link_input.py 保持一致
        clamped = (value - 307.0) / (1693.0 - 307.0) * 2.0 - 1.0
        if abs(clamped) < 0.05:
            clamped = 0.0  # Deadzone
        return clamped

    def run(self):
        if not self.setup_serial():
            return
        buffer = bytearray()
        last_ch4_normal = 1.0
        while True:
            try:
                byte = self.serial_conn.read(1)
                if byte:
                    byte = byte[0]
                    if not buffer and byte == 0x0F:
                        buffer.append(byte)
                    elif buffer:
                        buffer.append(byte)
                        if len(buffer) >= SBUS_FRAME_SIZE:
                            channels = self.parse_sbus(buffer)
                            if channels:
                                ch4 = channels[4]
                                ch4_normal = self.normalize_channel_value(ch4)
                                delta = ch4_normal - last_ch4_normal
                                # print("delta:", delta, "ch4_normal:", ch4_normal)
                                if delta > 0.5 and ch4_normal > 0.5:
                                    print(f"Starting controller with ch4_normal: {ch4_normal}")
                                    self.start_controller()
                                elif delta < -0.5 and ch4_normal < -0.5:
                                    print(f"Stopping controller with ch4_normal: {ch4_normal}")
                                    self.stop_controller()
                                last_ch4_normal = ch4_normal
                            buffer.clear()
                # time.sleep(0.0001)
            except KeyboardInterrupt:
                logging.info("Shutting down")
                self.stop_controller()
                break
            except Exception as e:
                logging.error(f"Error in loop: {e}")
                time.sleep(1)

if __name__ == '__main__':
    controller = RCController()
    controller.run()