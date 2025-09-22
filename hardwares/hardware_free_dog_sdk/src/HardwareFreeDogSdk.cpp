//
// Created for hardware_free_dog_sdk
//

#include "hardware_free_dog_sdk/HardwareFreeDogSdk.h"
#include <sensor_msgs/msg/joy.hpp>

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
    }

    return return_type::OK;
}

return_type HardwareFreeDogSdk::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    convertJointDataToFDSC();
    sendLowCmd();
    
    return return_type::OK;
}

CallbackReturn HardwareFreeDogSdk::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Starting Free Dog SDK hardware interface...");
    
    running_ = true;
    communication_thread_ = std::thread(&HardwareFreeDogSdk::communicationLoop, this);
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn HardwareFreeDogSdk::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Stopping Free Dog SDK hardware interface...");
    
    running_ = false;
    if (communication_thread_.joinable())
    {
        communication_thread_.join();
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

void HardwareFreeDogSdk::communicationLoop()
{
    while (running_)
    {
        updateLowStateData();
        
        if (use_high_level_)
        {
            updateHighStateData();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 1kHz communication
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
        low_state_.parseData(data);
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

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(HardwareFreeDogSdk, hardware_interface::SystemInterface)