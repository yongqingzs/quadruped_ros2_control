#!/usr/bin/env python3
"""
Record AMP motion data from OCS2 quadruped controller.

This script subscribes to AMPMotionData messages and records them in the format
required for AMP training.

Data format (61D per frame):
- Root position (3D): [x, y, z]
- Root orientation (4D): [qx, qy, qz, qw]
- Joint positions (12D): [FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, 
                          RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
- Foot positions in base frame (12D): [FL_x, FL_y, FL_z, FR_x, FR_y, FR_z,
                                        RL_x, RL_y, RL_z, RR_x, RR_y, RR_z]
- Linear velocity (3D): [vx, vy, vz]
- Angular velocity (3D): [wx, wy, wz]
- Joint velocities (12D)
- Foot velocities in base frame (12D)

Usage:
    ros2 run ocs2_quadruped_controller record_amp_motion.py --output trot --duration 10.0 --frequency 50
"""

import rclpy
from rclpy.node import Node
from control_input_msgs.msg import AMPMotionData
import numpy as np
import json
import argparse
from datetime import datetime
import os


class AMPMotionRecorder(Node):
    def __init__(self, output_name, duration, frequency=50.0, motion_weight=1.0):
        super().__init__('amp_motion_recorder')
        
        self.output_name = output_name
        self.duration = duration
        self.frame_duration = 1.0 / frequency
        self.motion_weight = motion_weight
        self.max_frames = int(duration * frequency)
        
        # Storage for recorded frames (61D per frame)
        self.frames = []
        self.recording = False
        self.start_time = None
        self.last_record_time = None
        
        # Subscribe to AMPMotionData
        self.subscription = self.create_subscription(
            AMPMotionData,
            '/amp_motion_data',
            self.amp_data_callback,
            10
        )
        
        self.get_logger().info(f'AMP Motion Recorder initialized')
        self.get_logger().info(f'Output: {output_name}.txt')
        self.get_logger().info(f'Duration: {duration}s')
        self.get_logger().info(f'Frame duration: {self.frame_duration}s ({frequency}Hz)')
        self.get_logger().info(f'Waiting for AMP motion data on /amp_motion_data...')
        
    def amp_data_callback(self, msg):
        """Callback to record AMPMotionData"""
        current_time = self.get_clock().now()
        
        # Start recording on first message
        if not self.recording:
            self.recording = True
            self.start_time = current_time
            self.last_record_time = current_time
            self.get_logger().info('Started recording!')
        
        # Check if duration exceeded
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            if self.frames:
                self.save_and_exit()
            return
        
        # Throttle recording to maintain frame duration
        time_since_last = (current_time - self.last_record_time).nanoseconds / 1e9
        if time_since_last < self.frame_duration * 0.9:  # Allow 10% tolerance
            return
        
        self.last_record_time = current_time
        
        # Construct frame (61D)
        frame = []
        
        # Root position (3D)
        frame.extend(msg.root_pos)
        
        # Root orientation (4D) - quaternion [x, y, z, w]
        frame.extend(msg.root_quat)
        
        # Joint positions (12D)
        frame.extend(msg.joint_pos)
        
        # Foot positions in base frame (12D)
        frame.extend(msg.foot_pos_local)
        
        # Linear velocity (3D)
        frame.extend(msg.linear_vel)
        
        # Angular velocity (3D)
        frame.extend(msg.angular_vel)
        
        # Joint velocities (12D)
        frame.extend(msg.joint_vel)
        
        # Foot velocities in base frame (12D)
        frame.extend(msg.foot_vel_local)
        
        self.frames.append(frame)
        
        # Log progress
        if len(self.frames) % 50 == 0:
            self.get_logger().info(f'Recorded {len(self.frames)} frames ({elapsed:.1f}s / {self.duration}s)')
    
    def save_and_exit(self):
        """Save recorded data to JSON file and exit"""
        output_dir = os.path.expanduser('~/amp_motion_data')
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = os.path.join(output_dir, f'{self.output_name}_{timestamp}.txt')
        
        # Prepare JSON data
        data = {
            "LoopMode": "Wrap",
            "FrameDuration": self.frame_duration,
            "EnableCycleOffsetPosition": False,
            "EnableCycleOffsetRotation": True,
            "MotionWeight": self.motion_weight,
            "Frames": self.frames
        }
        
        # Save to file
        with open(output_file, 'w') as f:
            json.dump(data, f, indent=2)
        
        self.get_logger().info(f'Saved {len(self.frames)} frames to {output_file}')
        self.get_logger().info(f'Recording completed! Frame count: {len(self.frames)}')
        self.get_logger().info(f'Duration: {len(self.frames) * self.frame_duration:.2f}s')
        
        # Shutdown
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='Record AMP motion data')
    parser.add_argument('--output', type=str, required=True,
                       help='Output filename (without extension)')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Recording duration in seconds')
    parser.add_argument('--frequency', type=float, default=50.0,
                       help='Recording frequency in Hz')
    parser.add_argument('--weight', type=float, default=1.0,
                       help='Motion weight for AMP sampling')
    
    parsed_args, remaining = parser.parse_known_args()
    
    rclpy.init(args=remaining)
    
    recorder = AMPMotionRecorder(
        output_name=parsed_args.output,
        duration=parsed_args.duration,
        frequency=parsed_args.frequency,
        motion_weight=parsed_args.weight
    )
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        if recorder.frames:
            recorder.save_and_exit()
    finally:
        recorder.destroy_node()


if __name__ == '__main__':
    main()
