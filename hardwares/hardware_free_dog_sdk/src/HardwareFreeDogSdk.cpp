//
// Created for hardware_free_dog_sdk
//

#include "hardware_free_dog_sdk/HardwareFreeDogSdk.h"
#include <sensor_msgs/msg/joy.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <chrono>
#include <iostream>

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn HardwareFreeDogSdk::on_init(const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Initialize command vectors
    joint_torque_command_.assign(12, 0);
    joint_position_command_.assign(12, 0);
    joint_velocities_command_.assign(12, 0);
    joint_kp_command_.assign(12, 0);
    joint_kd_command_.assign(12, 0);

    // Initialize state vectors
    joint_position_.assign(12, 0);
    joint_velocities_.assign(12, 0);
    joint_effort_.assign(12, 0);

    // Initialize sensor data
    imu_states_.assign(10, 0);  // quaternion(4) + gyro(3) + accel(3)
    foot_force_.assign(4, 0);
    high_states_.assign(6, 0);  // position(3) + velocity(3)

    // Parse joint interfaces
    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }

    // Parse hardware parameters
    if (const auto connection_param = info.hardware_parameters.find("connection_settings");
        connection_param != info.hardware_parameters.end())
    {
        connection_settings_ = connection_param->second;
    }

    if (const auto show_foot_force_param = info.hardware_parameters.find("show_foot_force");
        show_foot_force_param != info.hardware_parameters.end())
    {
        show_foot_force_ = show_foot_force_param->second == "true";
    }

    if (const auto use_high_level_param = info.hardware_parameters.find("use_high_level");
        use_high_level_param != info.hardware_parameters.end())
    {
        use_high_level_ = use_high_level_param->second == "true";
    }

    if (const auto power_limit_param = info.hardware_parameters.find("power_limit");
        power_limit_param != info.hardware_parameters.end())
    {
        power_limit_ = std::stod(power_limit_param->second);
    }

    RCLCPP_INFO(get_logger(), "connection_settings: %s, show_foot_force: %s, use_high_level: %s",
                connection_settings_.c_str(), show_foot_force_ ? "true" : "false",
                use_high_level_ ? "true" : "false");

    // Initialize Free Dog SDK connection
    try {
        udp_connection_ = std::make_shared<FDSC::UnitreeConnection>(connection_settings_);
        udp_connection_->startRecv();
        
        // Initialize low command
        initLowCmd();
        
        // Send initial command to establish connection
        std::vector<uint8_t> cmd_bytes = low_cmd_.buildCmd(false);
        udp_connection_->send(cmd_bytes);
        
        RCLCPP_INFO(get_logger(), "Free Dog SDK connection initialized successfully");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize Free Dog SDK connection: %s", e.what());
        return CallbackReturn::ERROR;
    }

    // Initialize ROS2 publishers
    auto node = rclcpp::Node::make_shared("hardware_free_dog_sdk_node");
    low_state_publisher_ = node->create_publisher<unitree_go::msg::LowState>("unitree_go/low_state", 10);
    low_cmd_publisher_ = node->create_publisher<unitree_go::msg::LowCmd>("unitree_go/low_cmd", 10);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HardwareFreeDogSdk::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Export joint state interfaces
    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    // Export IMU sensor state interfaces
    if (!info_.sensors.empty())
    {
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]);
        }
    }

    // Export foot force sensor state interfaces
    if (info_.sensors.size() > 1)
    {
        for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_force_[i]);
        }
    }

    // Export high-level state interfaces (odometer)
    if (info_.sensors.size() > 2)
    {
        for (uint i = 0; i < info_.sensors[2].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[2].name, info_.sensors[2].state_interfaces[i].name, &high_states_[i]);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareFreeDogSdk::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Export joint command interfaces
    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
        ind++;
    }

    return command_interfaces;
}

return_type HardwareFreeDogSdk::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Get latest data from Free Dog SDK
    std::vector<std::vector<uint8_t>> dataall;
    udp_connection_->getData(dataall);
    
    if (!dataall.empty())
    {
        // Use the latest received data
        std::vector<uint8_t> data = dataall.back();
        low_state_.parseData(data);
        
        convertJointDataFromFDSC();
        convertImuData();
        convertFootForceData();
        
        if (use_high_level_)
        {
            updateHighStateData();
        }

        // Publish LowState message
        publishLowState();
    }

    return return_type::OK;
}

return_type HardwareFreeDogSdk::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    convertJointDataToFDSC();
    sendLowCmd();

    // Publish LowCmd message
    publishLowCmd();
    
    return return_type::OK;
}

CallbackReturn HardwareFreeDogSdk::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Starting Free Dog SDK hardware interface...");
    
    // Start output thread
    stop_output_thread_ = false;
    output_thread_ = std::thread(&HardwareFreeDogSdk::outputValues, this);
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn HardwareFreeDogSdk::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Stopping Free Dog SDK hardware interface...");
    
    // Stop output thread
    stop_output_thread_ = true;
    if (output_thread_.joinable()) {
        output_thread_.join();
    }
    
    return CallbackReturn::SUCCESS;
}

void HardwareFreeDogSdk::initLowCmd()
{
    // Initialize low command structure
    low_cmd_.head = {0xFE, 0xEF};
    low_cmd_.levelFlag = 0xFF;  // Low-level control
    low_cmd_.frameReserve = 0;
    
    for (int i = 0; i < 20; i++)
    {
        low_cmd_.motorCmd.motors[i].mode = FDSC::MotorModeLow::Servo;  // Servo control mode
        low_cmd_.motorCmd.motors[i].q = 0.0f;
        low_cmd_.motorCmd.motors[i].dq = 0.0f;
        low_cmd_.motorCmd.motors[i].tau = 0.0f;
        low_cmd_.motorCmd.motors[i].Kp = 0.0f;
        low_cmd_.motorCmd.motors[i].Kd = 0.0f;
        low_cmd_.motorCmd.motors[i].reserve = {0.0f, 0.0f, 0.0f};
    }
}

void HardwareFreeDogSdk::updateLowStateData()
{
    std::vector<std::vector<uint8_t>> dataall;
    udp_connection_->getData(dataall);
    
    if (!dataall.empty())
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::vector<uint8_t> data = dataall.back();
        
        // Check data length before parsing (minimum expected size for LowState)
        const size_t min_expected_size = 100;  // Adjust based on FDSC::LowState structure
        if (data.size() >= min_expected_size)
        {
            try
            {
                low_state_.parseData(data);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Failed to parse low state data: %s", e.what());
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Received incomplete low state data packet, size: %zu (expected >= %zu)", 
                       data.size(), min_expected_size);
        }
    }
}

void HardwareFreeDogSdk::updateHighStateData()
{
    // Implementation would depend on high-level state data structure in free_dog_sdk_cpp
    // For now, this is a placeholder
}

void HardwareFreeDogSdk::sendLowCmd()
{
    std::vector<uint8_t> cmd_bytes = low_cmd_.buildCmd(false);
    udp_connection_->send(cmd_bytes);
}

void HardwareFreeDogSdk::convertJointDataFromFDSC()
{
    // Convert joint data from FDSC format to ROS2 Control format
    for (int i = 0; i < 12; ++i)
    {
        joint_position_[i] = low_state_.motorState[i].q;
        joint_velocities_[i] = low_state_.motorState[i].dq;
        joint_effort_[i] = low_state_.motorState[i].tauEst;
    }
}

void HardwareFreeDogSdk::convertJointDataToFDSC()
{
    // Convert joint commands from ROS2 Control format to FDSC format
    for (int i = 0; i < 12; ++i)
    {
        low_cmd_.motorCmd.motors[i].q = static_cast<float>(joint_position_command_[i]);
        low_cmd_.motorCmd.motors[i].dq = static_cast<float>(joint_velocities_command_[i]);
        low_cmd_.motorCmd.motors[i].tau = static_cast<float>(joint_torque_command_[i]);
        low_cmd_.motorCmd.motors[i].Kp = static_cast<float>(joint_kp_command_[i]);
        low_cmd_.motorCmd.motors[i].Kd = static_cast<float>(joint_kd_command_[i]);
    }
}

void HardwareFreeDogSdk::convertImuData()
{
    // Convert IMU data from FDSC format to ROS2 Control format
    // Quaternion (w, x, y, z)
    imu_states_[0] = low_state_.imu_quaternion[0];  // w
    imu_states_[1] = low_state_.imu_quaternion[1];  // x
    imu_states_[2] = low_state_.imu_quaternion[2];  // y
    imu_states_[3] = low_state_.imu_quaternion[3];  // z
    
    // Angular velocity
    imu_states_[4] = low_state_.imu_gyroscope[0];
    imu_states_[5] = low_state_.imu_gyroscope[1];
    imu_states_[6] = low_state_.imu_gyroscope[2];
    
    // Linear acceleration
    imu_states_[7] = low_state_.imu_accelerometer[0];
    imu_states_[8] = low_state_.imu_accelerometer[1];
    imu_states_[9] = low_state_.imu_accelerometer[2];
}

void HardwareFreeDogSdk::convertFootForceData()
{
    // Convert foot force data from FDSC format to ROS2 Control format
    for (int i = 0; i < 4; ++i)
    {
        foot_force_[i] = low_state_.footForce[i];
    }
    
    if (show_foot_force_)
    {
        RCLCPP_INFO(get_logger(), "foot_force: %f, %f, %f, %f", 
                    foot_force_[0], foot_force_[1], foot_force_[2], foot_force_[3]);
    }
}

void HardwareFreeDogSdk::outputValues()
{
    rclcpp::Rate rate(0.5);  // 0.5 Hz = every 2 seconds
    while (!stop_output_thread_ && rclcpp::ok())
    {
        std::cout << "=== Read Values ===" << std::endl;
        for (int i = 0; i < 12; ++i) {
            std::cout << "Read: joint [" << i << "]: pos " << joint_position_[i] 
                      << ", vel " << joint_velocities_[i] << ", tau " << joint_effort_[i] << std::endl;
        }
        std::cout << "Read: IMU ori: [" << imu_states_[0] << ", " << imu_states_[1] << ", " 
                  << imu_states_[2] << ", " << imu_states_[3] << "]" << std::endl;
        std::cout << "Read: IMU ang_vel: [" << imu_states_[4] << ", " << imu_states_[5] << ", " 
                  << imu_states_[6] << "]" << std::endl;
        std::cout << "Read: IMU lin_acc: [" << imu_states_[7] << ", " << imu_states_[8] << ", " 
                  << imu_states_[9] << "]" << std::endl;

        std::cout << "=== Write Values ===" << std::endl;
        for (int i = 0; i < 12; ++i) {
            std::cout << "Write: joint [" << i << "]: q " << low_cmd_.motorCmd.motors[i].q 
                      << ", dq " << low_cmd_.motorCmd.motors[i].dq 
                      << ", kp " << low_cmd_.motorCmd.motors[i].Kp 
                      << ", kd " << low_cmd_.motorCmd.motors[i].Kd 
                      << ", tau " << low_cmd_.motorCmd.motors[i].tau << std::endl;
        }

        rate.sleep();
    }
}

void HardwareFreeDogSdk::publishLowState()
{
    auto low_state_msg = unitree_go::msg::LowState();
    
    // Fill IMU data
    for (int i = 0; i < 4; ++i) {
        low_state_msg.imu_state.quaternion[i] = imu_states_[i];
    }
    for (int i = 0; i < 3; ++i) {
        low_state_msg.imu_state.gyroscope[i] = imu_states_[4 + i];
        low_state_msg.imu_state.accelerometer[i] = imu_states_[7 + i];
    }
    low_state_msg.imu_state.temperature = low_state_.temperature_imu;

    // Fill motor states (only first 12 motors for quadruped)
    for (int i = 0; i < 12; ++i) {
        low_state_msg.motor_state[i].mode = low_state_.motorState[i].mode;
        low_state_msg.motor_state[i].q = low_state_.motorState[i].q;
        low_state_msg.motor_state[i].dq = low_state_.motorState[i].dq;
        low_state_msg.motor_state[i].ddq = low_state_.motorState[i].ddq;
        low_state_msg.motor_state[i].tau_est = low_state_.motorState[i].tauEst;
        low_state_msg.motor_state[i].q_raw = low_state_.motorState[i].q_raw;
        low_state_msg.motor_state[i].dq_raw = low_state_.motorState[i].dq_raw;
        low_state_msg.motor_state[i].ddq_raw = low_state_.motorState[i].ddq_raw;
        low_state_msg.motor_state[i].temperature = low_state_.motorState[i].temperature;
    }

    // Fill BMS data
    low_state_msg.bms_state.version_high = low_state_.version_h;
    low_state_msg.bms_state.version_low = low_state_.version_l;
    low_state_msg.bms_state.status = low_state_.bms_status;
    low_state_msg.bms_state.soc = low_state_.SOC;

    // Fill foot force data
    for (int i = 0; i < 4; ++i) {
        low_state_msg.foot_force[i] = low_state_.footForce[i];
        low_state_msg.foot_force_est[i] = low_state_.footForceEst[i];
    }

    // Fill wireless remote data
    for (int i = 0; i < 40; ++i) {
        low_state_msg.wireless_remote[i] = low_state_.wirelessRemote[i];
    }

    low_state_publisher_->publish(low_state_msg);
}

void HardwareFreeDogSdk::publishLowCmd()
{
    auto low_cmd_msg = unitree_go::msg::LowCmd();
    
    // Fill motor commands (only first 12 motors for quadruped)
    for (int i = 0; i < 12; ++i) {
        low_cmd_msg.motor_cmd[i].mode = static_cast<uint8_t>(low_cmd_.motorCmd.motors[i].mode);
        low_cmd_msg.motor_cmd[i].q = low_cmd_.motorCmd.motors[i].q;
        low_cmd_msg.motor_cmd[i].dq = low_cmd_.motorCmd.motors[i].dq;
        low_cmd_msg.motor_cmd[i].tau = low_cmd_.motorCmd.motors[i].tau;
        low_cmd_msg.motor_cmd[i].kp = low_cmd_.motorCmd.motors[i].Kp;
        low_cmd_msg.motor_cmd[i].kd = low_cmd_.motorCmd.motors[i].Kd;
    }

    // Fill wireless remote data (copy from lowState as it's typically echoed back)
    for (int i = 0; i < 40; ++i) {
        low_cmd_msg.wireless_remote[i] = low_state_.wirelessRemote[i];
    }

    low_cmd_publisher_->publish(low_cmd_msg);
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(HardwareFreeDogSdk, hardware_interface::SystemInterface)