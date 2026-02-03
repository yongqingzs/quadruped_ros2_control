# AMPMotionData message for recording AMP training data
# Total: 61 dimensions per frame

# Root state (7D)
float64[3] root_pos          # Root position in world frame [x, y, z]
float64[4] root_quat         # Root orientation quaternion [x, y, z, w]

# Joint positions (12D)
float64[12] joint_pos        # Joint positions [FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]

# Foot positions in base frame (12D)
float64[12] foot_pos_local   # Foot positions in base frame [FL_x, FL_y, FL_z, FR_x, FR_y, FR_z, RL_x, RL_y, RL_z, RR_x, RR_y, RR_z]

# Base velocities (6D)
float64[3] linear_vel        # Linear velocity in world frame [vx, vy, vz]
float64[3] angular_vel       # Angular velocity in base frame [wx, wy, wz]

# Joint velocities (12D)
float64[12] joint_vel        # Joint velocities

# Foot velocities in base frame (12D)
float64[12] foot_vel_local   # Foot velocities in base frame

# Metadata
builtin_interfaces/Time timestamp
