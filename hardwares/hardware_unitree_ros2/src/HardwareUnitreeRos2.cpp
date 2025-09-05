//
// Created for hardware_unitree_ros2 package
//

#include "hardware_unitree_ros2/HardwareUnitreeRos2.h"

using hardware_interface::return_type;

// Implementation of Ros2CommunicationBridge
HardwareUnitreeRos2::Ros2CommunicationBridge::Ros2CommunicationBridge()
{
    // Initialize the ROS2 node for this bridge
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    node_ = rclcpp::Node::make_shared("hardware_unitree_ros2_bridge", options);
}

HardwareUnitreeRos2::Ros2CommunicationBridge::~Ros2CommunicationBridge()
{
    shutdown();
}

bool HardwareUnitreeRos2::Ros2CommunicationBridge::initialize()
{
    try
    {
        // Create publisher for low commands
        low_cmd_publisher_ = node_->create_publisher<unitree_go::msg::LowCmd>(
            "unitree_go/low_cmd", rclcpp::QoS(10).reliable());

        RCLCPP_INFO(node_->get_logger(), "Hardware Unitree ROS2 bridge initialized successfully");
        
        // Start the spinning thread
        active_ = true;
        spin_thread_ = std::make_unique<std::thread>(&Ros2CommunicationBridge::spinThread, this);
        
        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize bridge: %s", e.what());
        return false;
    }
}

void HardwareUnitreeRos2::Ros2CommunicationBridge::shutdown()
{
    active_ = false;
    
    if (spin_thread_ && spin_thread_->joinable())
    {
        spin_thread_->join();
    }
    
    // Reset publishers and subscribers
    low_cmd_publisher_.reset();
    low_state_subscriber_.reset();
    high_state_subscriber_.reset();
}

void HardwareUnitreeRos2::Ros2CommunicationBridge::publishLowCmd(const unitree_go::msg::LowCmd& cmd)
{
    if (low_cmd_publisher_ && active_)
    {
        low_cmd_publisher_->publish(cmd);
    }
}

void HardwareUnitreeRos2::Ros2CommunicationBridge::setStateCallback(
    std::function<void(const unitree_go::msg::LowState::SharedPtr)> callback)
{
    low_state_subscriber_ = node_->create_subscription<unitree_go::msg::LowState>(
        "unitree_go/low_state", rclcpp::QoS(10).reliable(), callback);
}

void HardwareUnitreeRos2::Ros2CommunicationBridge::setHighStateCallback(
    std::function<void(const unitree_go::msg::SportModeState::SharedPtr)> callback)
{
    high_state_subscriber_ = node_->create_subscription<unitree_go::msg::SportModeState>(
        "unitree_go/high_state", rclcpp::QoS(10).reliable(), callback);
}

void HardwareUnitreeRos2::Ros2CommunicationBridge::spinThread()
{
    while (active_ && rclcpp::ok())
    {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// Implementation of HardwareUnitreeRos2
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitreeRos2::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    joint_torque_command_.assign(12, 0);
    joint_position_command_.assign(12, 0);
    joint_velocities_command_.assign(12, 0);
    joint_kp_command_.assign(12, 0);
    joint_kd_command_.assign(12, 0);

    joint_position_.assign(12, 0);
    joint_velocities_.assign(12, 0);
    joint_effort_.assign(12, 0);

    imu_states_.assign(10, 0);
    foot_force_.assign(4, 0);
    high_states_.assign(6, 0);

    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }

    if (const auto show_foot_force_param = info.hardware_parameters.find("show_foot_force"); 
        show_foot_force_param != info.hardware_parameters.end())
    {
        show_foot_force_ = show_foot_force_param->second == "true";
    }

    RCLCPP_INFO(get_logger(), "HardwareUnitreeRos2 initialized with show_foot_force: %s", 
                show_foot_force_ ? "true" : "false");

    return SystemInterface::on_init(info);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitreeRos2::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Activating HardwareUnitreeRos2...");

    // Initialize ROS2 communication bridge
    try
    {
        initializeRos2Bridge();
        initLowCmd();

        communication_active_ = true;
        last_state_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(), "HardwareUnitreeRos2 activated successfully!");
        return CallbackReturn::SUCCESS;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to activate HardwareUnitreeRos2: %s", e.what());
        return CallbackReturn::ERROR;
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitreeRos2::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating HardwareUnitreeRos2...");

    communication_active_ = false;
    shutdownRos2Bridge();

    RCLCPP_INFO(get_logger(), "HardwareUnitreeRos2 deactivated successfully!");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HardwareUnitreeRos2::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

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

    // export imu sensor state interface
    if (!info_.sensors.empty())
    {
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]);
        }
    }

    // export foot force sensor state interface
    if (info_.sensors.size() > 1)
    {
        for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_force_[i]);
        }
    }

    // export odometer state interface
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

std::vector<hardware_interface::CommandInterface> HardwareUnitreeRos2::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

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

return_type HardwareUnitreeRos2::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Check for communication timeout
    auto now = std::chrono::steady_clock::now();
    auto time_since_last_state = std::chrono::duration_cast<std::chrono::duration<double>>(
        now - last_state_time_).count();
    
    if (communication_active_ && time_since_last_state > STATE_TIMEOUT_SEC)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                            "State communication timeout (%.2f s)", time_since_last_state);
    }

    // joint states
    for (int i(0); i < 12; ++i)
    {
        if (i < 20) // motor_state is MotorState[20] array
        {
            joint_position_[i] = low_state_.motor_state[i].q;
            joint_velocities_[i] = low_state_.motor_state[i].dq;
            joint_effort_[i] = low_state_.motor_state[i].tau_est;
        }
    }

    // imu states
    if (low_state_.imu_state.quaternion.size() >= 4)
    {
        imu_states_[0] = low_state_.imu_state.quaternion[0]; // w
        imu_states_[1] = low_state_.imu_state.quaternion[1]; // x
        imu_states_[2] = low_state_.imu_state.quaternion[2]; // y
        imu_states_[3] = low_state_.imu_state.quaternion[3]; // z
    }
    
    if (low_state_.imu_state.gyroscope.size() >= 3)
    {
        imu_states_[4] = low_state_.imu_state.gyroscope[0];
        imu_states_[5] = low_state_.imu_state.gyroscope[1];
        imu_states_[6] = low_state_.imu_state.gyroscope[2];
    }
    
    if (low_state_.imu_state.accelerometer.size() >= 3)
    {
        imu_states_[7] = low_state_.imu_state.accelerometer[0];
        imu_states_[8] = low_state_.imu_state.accelerometer[1];
        imu_states_[9] = low_state_.imu_state.accelerometer[2];
    }

    // contact states (foot_force)
    if (low_state_.foot_force.size() >= 4)
    {
        foot_force_[0] = low_state_.foot_force[0];
        foot_force_[1] = low_state_.foot_force[1];
        foot_force_[2] = low_state_.foot_force[2];
        foot_force_[3] = low_state_.foot_force[3];

        if (show_foot_force_)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                               "foot_force(): %f, %f, %f, %f", 
                               foot_force_[0], foot_force_[1], foot_force_[2], foot_force_[3]);
        }
    }

    // high states (position and velocity)
    if (high_state_.position.size() >= 3)
    {
        high_states_[0] = high_state_.position[0];
        high_states_[1] = high_state_.position[1];
        high_states_[2] = high_state_.position[2];
    }
    
    if (high_state_.velocity.size() >= 3)
    {
        high_states_[3] = high_state_.velocity[0];
        high_states_[4] = high_state_.velocity[1];
        high_states_[5] = high_state_.velocity[2];
    }

    return return_type::OK;
}

return_type HardwareUnitreeRos2::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    
    if (!communication_active_ || !ros2_bridge_ || !ros2_bridge_->isActive())
    {
        return return_type::OK;
    }

    // Prepare command message
    for (int i(0); i < 12; ++i)
    {
        if (i < 20) // motor_cmd is MotorCmd[20] array
        {
            low_cmd_.motor_cmd[i].mode = 0x01;
            low_cmd_.motor_cmd[i].q = static_cast<float>(joint_position_command_[i]);
            low_cmd_.motor_cmd[i].dq = static_cast<float>(joint_velocities_command_[i]);
            low_cmd_.motor_cmd[i].kp = static_cast<float>(joint_kp_command_[i]);
            low_cmd_.motor_cmd[i].kd = static_cast<float>(joint_kd_command_[i]);
            low_cmd_.motor_cmd[i].tau = static_cast<float>(joint_torque_command_[i]);
        }
    }

    // Set header info (head is uint8[2])
    low_cmd_.head[0] = 0xFE;
    low_cmd_.head[1] = 0xEF;
    low_cmd_.level_flag = 0xFF;
    low_cmd_.gpio = 0;

    // Publish command via ROS2
    ros2_bridge_->publishLowCmd(low_cmd_);
    
    return return_type::OK;
}

void HardwareUnitreeRos2::initLowCmd()
{
    // Initialize header (head is uint8[2])
    low_cmd_.head[0] = 0xFE;
    low_cmd_.head[1] = 0xEF;
    low_cmd_.level_flag = 0xFF;
    low_cmd_.gpio = 0;

    // Initialize motor commands for 20 motors (motor_cmd is MotorCmd[20] array)
    for (int i = 0; i < 20; i++)
    {
        low_cmd_.motor_cmd[i].mode = 0x01; // motor switch to servo (PMSM) mode
        low_cmd_.motor_cmd[i].q = 0;
        low_cmd_.motor_cmd[i].kp = 0;
        low_cmd_.motor_cmd[i].dq = 0;
        low_cmd_.motor_cmd[i].kd = 0;
        low_cmd_.motor_cmd[i].tau = 0;
    }
}

void HardwareUnitreeRos2::initializeRos2Bridge()
{
    ros2_bridge_ = std::make_unique<Ros2CommunicationBridge>();
    
    if (!ros2_bridge_->initialize())
    {
        throw std::runtime_error("Failed to initialize ROS2 communication bridge");
    }

    // Set up callbacks for state reception
    ros2_bridge_->setStateCallback(
        std::bind(&HardwareUnitreeRos2::lowStateCallback, this, std::placeholders::_1));
    
    ros2_bridge_->setHighStateCallback(
        std::bind(&HardwareUnitreeRos2::highStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ROS2 communication bridge initialized");
}

void HardwareUnitreeRos2::shutdownRos2Bridge()
{
    if (ros2_bridge_)
    {
        ros2_bridge_->shutdown();
        ros2_bridge_.reset();
    }
}

void HardwareUnitreeRos2::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    low_state_ = *msg;
    last_state_time_ = std::chrono::steady_clock::now();
}

void HardwareUnitreeRos2::highStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    high_state_ = *msg;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    HardwareUnitreeRos2, hardware_interface::SystemInterface)
