//
// DDS-ROS2 Bridge Node Implementation for hardware_unitree_ros2
//

#include "hardware_unitree_ros2/dds_ros2_bridge_node.h"
#include <unitree/robot/channel/channel_factory.hpp>

using namespace unitree::robot;

namespace hardware_unitree_ros2 {

DdsRos2BridgeNode::DdsRos2BridgeNode(const rclcpp::NodeOptions& options)
    : Node("dds_ros2_bridge_node", options)
{
    // Declare parameters
    declare_parameter("network_interface", "lo");
    declare_parameter("domain", 1);
    declare_parameter("debug_output", false);

    // Get parameters
    network_interface_ = get_parameter("network_interface").as_string();
    domain_ = get_parameter("domain").as_int();
    debug_output_ = get_parameter("debug_output").as_bool();

    RCLCPP_INFO(get_logger(), "DDS-ROS2 Bridge Node starting with interface: %s, domain: %d", 
                network_interface_.c_str(), domain_);
}

DdsRos2BridgeNode::~DdsRos2BridgeNode()
{
    shutdown();
}

bool DdsRos2BridgeNode::initialize()
{
    try {
        // Initialize DDS channel factory
        ChannelFactory::Instance()->Init(domain_, network_interface_);

        // Create DDS publishers and subscribers
        dds_low_cmd_publisher_ = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
        dds_low_cmd_publisher_->InitChannel();

        dds_low_state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
        dds_low_state_subscriber_->InitChannel(
            [this](auto&& msg) { ddsLowStateHandler(std::forward<decltype(msg)>(msg)); }, 1);

        dds_high_state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(TOPIC_HIGHSTATE);
        dds_high_state_subscriber_->InitChannel(
            [this](auto&& msg) { ddsHighStateHandler(std::forward<decltype(msg)>(msg)); }, 1);

        // Create ROS2 publishers and subscribers
        auto qos = rclcpp::QoS(10).reliable();
        
        ros2_low_state_publisher_ = create_publisher<unitree_go::msg::LowState>(ROS2_TOPIC_LOWSTATE, qos);
        ros2_high_state_publisher_ = create_publisher<unitree_go::msg::SportModeState>(ROS2_TOPIC_HIGHSTATE, qos);
        
        ros2_low_cmd_subscriber_ = create_subscription<unitree_go::msg::LowCmd>(
            ROS2_TOPIC_LOWCMD, qos,
            [this](const unitree_go::msg::LowCmd::SharedPtr msg) { ros2LowCmdHandler(msg); });

        active_ = true;
        
        RCLCPP_INFO(get_logger(), "DDS-ROS2 Bridge Node initialized successfully");
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize DDS-ROS2 Bridge: %s", e.what());
        return false;
    }
}

void DdsRos2BridgeNode::shutdown()
{
    active_ = false;
    
    // Reset DDS publishers and subscribers
    dds_low_cmd_publisher_.reset();
    dds_low_state_subscriber_.reset();
    dds_high_state_subscriber_.reset();
    
    // Reset ROS2 publishers and subscribers
    ros2_low_state_publisher_.reset();
    ros2_high_state_publisher_.reset();
    ros2_low_cmd_subscriber_.reset();
    
    RCLCPP_INFO(get_logger(), "DDS-ROS2 Bridge Node shutdown complete");
}

void DdsRos2BridgeNode::ddsLowStateHandler(const void* message)
{
    if (!active_) return;
    
    try {
        const auto& dds_low_state = *static_cast<const unitree_go::msg::dds_::LowState_*>(message);
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        auto ros2_low_state = std::make_unique<unitree_go::msg::LowState>();
        convertDdsToRos2LowState(dds_low_state, *ros2_low_state);
        
        if (ros2_low_state_publisher_) {
            ros2_low_state_publisher_->publish(*ros2_low_state);
        }
        
        if (debug_output_) {
            RCLCPP_DEBUG(get_logger(), "Converted and published DDS LowState to ROS2");
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in DDS LowState handler: %s", e.what());
    }
}

void DdsRos2BridgeNode::ddsHighStateHandler(const void* message)
{
    if (!active_) return;
    
    try {
        const auto& dds_high_state = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        auto ros2_high_state = std::make_unique<unitree_go::msg::SportModeState>();
        convertDdsToRos2SportModeState(dds_high_state, *ros2_high_state);
        
        if (ros2_high_state_publisher_) {
            ros2_high_state_publisher_->publish(*ros2_high_state);
        }
        
        if (debug_output_) {
            RCLCPP_DEBUG(get_logger(), "Converted and published DDS SportModeState to ROS2");
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in DDS SportModeState handler: %s", e.what());
    }
}

void DdsRos2BridgeNode::ros2LowCmdHandler(const unitree_go::msg::LowCmd::SharedPtr msg)
{
    if (!active_) return;
    
    try {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        unitree_go::msg::dds_::LowCmd_ dds_low_cmd;
        convertRos2ToDdsLowCmd(*msg, dds_low_cmd);
        
        // Calculate CRC (similar to hardware_unitree_sdk2)
        dds_low_cmd.crc() = crc32_core(reinterpret_cast<uint32_t*>(&dds_low_cmd),
                                       (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        
        if (dds_low_cmd_publisher_) {
            dds_low_cmd_publisher_->Write(dds_low_cmd);
        }
        
        if (debug_output_) {
            RCLCPP_DEBUG(get_logger(), "Converted and published ROS2 LowCmd to DDS");
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in ROS2 LowCmd handler: %s", e.what());
    }
}

void DdsRos2BridgeNode::convertDdsToRos2LowState(const unitree_go::msg::dds_::LowState_& dds_msg, 
                                                  unitree_go::msg::LowState& ros2_msg)
{
    // Header
    ros2_msg.head = {dds_msg.head()[0], dds_msg.head()[1]};
    ros2_msg.level_flag = dds_msg.level_flag();
    ros2_msg.frame_reserve = dds_msg.frame_reserve();
    ros2_msg.sn = {dds_msg.sn()[0], dds_msg.sn()[1]};
    ros2_msg.version = {dds_msg.version()[0], dds_msg.version()[1]};
    ros2_msg.bandwidth = dds_msg.bandwidth();
    
    // IMU state
    ros2_msg.imu_state.quaternion = {
        dds_msg.imu_state().quaternion()[0],
        dds_msg.imu_state().quaternion()[1], 
        dds_msg.imu_state().quaternion()[2],
        dds_msg.imu_state().quaternion()[3]
    };
    ros2_msg.imu_state.gyroscope = {
        dds_msg.imu_state().gyroscope()[0],
        dds_msg.imu_state().gyroscope()[1],
        dds_msg.imu_state().gyroscope()[2]
    };
    ros2_msg.imu_state.accelerometer = {
        dds_msg.imu_state().accelerometer()[0],
        dds_msg.imu_state().accelerometer()[1],
        dds_msg.imu_state().accelerometer()[2]
    };
    ros2_msg.imu_state.rpy = {
        dds_msg.imu_state().rpy()[0],
        dds_msg.imu_state().rpy()[1],
        dds_msg.imu_state().rpy()[2]
    };
    ros2_msg.imu_state.temperature = dds_msg.imu_state().temperature();
    
    // Motor states (motor_state is MotorState[20] array, not vector)
    for (int i = 0; i < 20; ++i) {
        unitree_go::msg::MotorState motor_state;
        motor_state.mode = dds_msg.motor_state()[i].mode();
        motor_state.q = dds_msg.motor_state()[i].q();
        motor_state.dq = dds_msg.motor_state()[i].dq();
        motor_state.ddq = dds_msg.motor_state()[i].ddq();
        motor_state.tau_est = dds_msg.motor_state()[i].tau_est();
        motor_state.q_raw = dds_msg.motor_state()[i].q_raw();
        motor_state.dq_raw = dds_msg.motor_state()[i].dq_raw();
        motor_state.ddq_raw = dds_msg.motor_state()[i].ddq_raw();
        motor_state.temperature = dds_msg.motor_state()[i].temperature();
        motor_state.lost = dds_msg.motor_state()[i].lost();
        motor_state.reserve = {
            dds_msg.motor_state()[i].reserve()[0],
            dds_msg.motor_state()[i].reserve()[1]
        };
        ros2_msg.motor_state[i] = motor_state;
    }
    
    // BMS state
    ros2_msg.bms_state.version_high = dds_msg.bms_state().version_high();
    ros2_msg.bms_state.version_low = dds_msg.bms_state().version_low();
    ros2_msg.bms_state.status = dds_msg.bms_state().status();  // Note: ROS2 uses 'status', not 'bms_status'
    ros2_msg.bms_state.soc = dds_msg.bms_state().soc();
    ros2_msg.bms_state.current = dds_msg.bms_state().current();
    ros2_msg.bms_state.cycle = dds_msg.bms_state().cycle();
    for (int i = 0; i < 2; ++i) {
        ros2_msg.bms_state.bq_ntc[i] = dds_msg.bms_state().bq_ntc()[i];
        ros2_msg.bms_state.mcu_ntc[i] = dds_msg.bms_state().mcu_ntc()[i];
    }
    for (int i = 0; i < 15; ++i) {  // Note: ROS2 BmsState has cell_vol[15], not [10]
        ros2_msg.bms_state.cell_vol[i] = dds_msg.bms_state().cell_vol()[i];
    }
    
    // Foot force and contact
    for (int i = 0; i < 4; ++i) {
        ros2_msg.foot_force[i] = dds_msg.foot_force()[i];
        ros2_msg.foot_force_est[i] = dds_msg.foot_force_est()[i];
    }
    
    // Tick and CRC
    ros2_msg.tick = dds_msg.tick();
    ros2_msg.wireless_remote = {
        dds_msg.wireless_remote()[0],
        dds_msg.wireless_remote()[1],
        dds_msg.wireless_remote()[2],
        dds_msg.wireless_remote()[3],
        dds_msg.wireless_remote()[4],
        dds_msg.wireless_remote()[5],
        dds_msg.wireless_remote()[6],
        dds_msg.wireless_remote()[7],
        dds_msg.wireless_remote()[8],
        dds_msg.wireless_remote()[9],
        dds_msg.wireless_remote()[10],
        dds_msg.wireless_remote()[11],
        dds_msg.wireless_remote()[12],
        dds_msg.wireless_remote()[13],
        dds_msg.wireless_remote()[14],
        dds_msg.wireless_remote()[15],
        dds_msg.wireless_remote()[16],
        dds_msg.wireless_remote()[17],
        dds_msg.wireless_remote()[18],
        dds_msg.wireless_remote()[19],
        dds_msg.wireless_remote()[20],
        dds_msg.wireless_remote()[21],
        dds_msg.wireless_remote()[22],
        dds_msg.wireless_remote()[23],
        dds_msg.wireless_remote()[24],
        dds_msg.wireless_remote()[25],
        dds_msg.wireless_remote()[26],
        dds_msg.wireless_remote()[27],
        dds_msg.wireless_remote()[28],
        dds_msg.wireless_remote()[29],
        dds_msg.wireless_remote()[30],
        dds_msg.wireless_remote()[31],
        dds_msg.wireless_remote()[32],
        dds_msg.wireless_remote()[33],
        dds_msg.wireless_remote()[34],
        dds_msg.wireless_remote()[35],
        dds_msg.wireless_remote()[36],
        dds_msg.wireless_remote()[37],
        dds_msg.wireless_remote()[38],
        dds_msg.wireless_remote()[39]
    };
    ros2_msg.bit_flag = dds_msg.bit_flag();
    ros2_msg.adc_reel = dds_msg.adc_reel();
    ros2_msg.temperature_ntc1 = dds_msg.temperature_ntc1();
    ros2_msg.temperature_ntc2 = dds_msg.temperature_ntc2();
    ros2_msg.power_v = dds_msg.power_v();
    ros2_msg.power_a = dds_msg.power_a();
    for (int i = 0; i < 4; ++i) {
        ros2_msg.fan_frequency[i] = dds_msg.fan_frequency()[i];
    }
    ros2_msg.reserve = dds_msg.reserve();
    ros2_msg.crc = dds_msg.crc();
}

void DdsRos2BridgeNode::convertDdsToRos2SportModeState(const unitree_go::msg::dds_::SportModeState_& dds_msg, 
                                                       unitree_go::msg::SportModeState& ros2_msg)
{
    ros2_msg.stamp.sec = dds_msg.stamp().sec();
    ros2_msg.stamp.nanosec = dds_msg.stamp().nanosec();
    ros2_msg.error_code = dds_msg.error_code();
    
    // IMU state conversion (reuse from LowState conversion logic)
    ros2_msg.imu_state.quaternion = {
        dds_msg.imu_state().quaternion()[0],
        dds_msg.imu_state().quaternion()[1], 
        dds_msg.imu_state().quaternion()[2],
        dds_msg.imu_state().quaternion()[3]
    };
    ros2_msg.imu_state.gyroscope = {
        dds_msg.imu_state().gyroscope()[0],
        dds_msg.imu_state().gyroscope()[1],
        dds_msg.imu_state().gyroscope()[2]
    };
    ros2_msg.imu_state.accelerometer = {
        dds_msg.imu_state().accelerometer()[0],
        dds_msg.imu_state().accelerometer()[1],
        dds_msg.imu_state().accelerometer()[2]
    };
    ros2_msg.imu_state.rpy = {
        dds_msg.imu_state().rpy()[0],
        dds_msg.imu_state().rpy()[1],
        dds_msg.imu_state().rpy()[2]
    };
    ros2_msg.imu_state.temperature = dds_msg.imu_state().temperature();
    
    ros2_msg.mode = dds_msg.mode();
    ros2_msg.progress = dds_msg.progress();
    ros2_msg.gait_type = dds_msg.gait_type();
    ros2_msg.foot_raise_height = dds_msg.foot_raise_height();
    for (int i = 0; i < 3; ++i) {
        ros2_msg.position[i] = dds_msg.position()[i];
        ros2_msg.velocity[i] = dds_msg.velocity()[i];
        ros2_msg.range_obstacle[i] = dds_msg.range_obstacle()[i];
    }
    ros2_msg.body_height = dds_msg.body_height();  // Note: ROS2 SportModeState has single body_height, not array
    ros2_msg.yaw_speed = dds_msg.yaw_speed();
    for (int i = 0; i < 4; ++i) {
        ros2_msg.foot_force[i] = dds_msg.foot_force()[i];
    }
    for (int i = 0; i < 12; ++i) {
        ros2_msg.foot_position_body[i] = dds_msg.foot_position_body()[i];
        ros2_msg.foot_speed_body[i] = dds_msg.foot_speed_body()[i];
    }
}

void DdsRos2BridgeNode::convertRos2ToDdsLowCmd(const unitree_go::msg::LowCmd& ros2_msg, 
                                               unitree_go::msg::dds_::LowCmd_& dds_msg)
{
    // Header
    dds_msg.head()[0] = ros2_msg.head[0];
    dds_msg.head()[1] = ros2_msg.head[1];
    dds_msg.level_flag() = ros2_msg.level_flag;
    dds_msg.frame_reserve() = ros2_msg.frame_reserve;
    dds_msg.sn()[0] = ros2_msg.sn[0];
    dds_msg.sn()[1] = ros2_msg.sn[1];
    dds_msg.version()[0] = ros2_msg.version[0];
    dds_msg.version()[1] = ros2_msg.version[1];
    dds_msg.bandwidth() = ros2_msg.bandwidth;
    
    // Motor commands
    for (int i = 0; i < 20 && i < 20; ++i) {  // Both are fixed arrays of size 20
        dds_msg.motor_cmd()[i].mode() = ros2_msg.motor_cmd[i].mode;
        dds_msg.motor_cmd()[i].q() = ros2_msg.motor_cmd[i].q;
        dds_msg.motor_cmd()[i].dq() = ros2_msg.motor_cmd[i].dq;
        dds_msg.motor_cmd()[i].tau() = ros2_msg.motor_cmd[i].tau;
        dds_msg.motor_cmd()[i].kp() = ros2_msg.motor_cmd[i].kp;
        dds_msg.motor_cmd()[i].kd() = ros2_msg.motor_cmd[i].kd;
        dds_msg.motor_cmd()[i].reserve()[0] = ros2_msg.motor_cmd[i].reserve[0];
        dds_msg.motor_cmd()[i].reserve()[1] = ros2_msg.motor_cmd[i].reserve[1];
        dds_msg.motor_cmd()[i].reserve()[2] = ros2_msg.motor_cmd[i].reserve[2];
    }
    
    // BMS command
    dds_msg.bms_cmd().off() = ros2_msg.bms_cmd.off;
    for (int i = 0; i < 3; ++i) {
        dds_msg.bms_cmd().reserve()[i] = ros2_msg.bms_cmd.reserve[i];
    }
    
    // Wireless remote
    for (int i = 0; i < 40 && i < 40; ++i) {  // Both are fixed arrays of size 40
        dds_msg.wireless_remote()[i] = ros2_msg.wireless_remote[i];
    }
    
    // LED (both are uint8[12] arrays, not LED objects)
    for (int i = 0; i < 12; ++i) {
        dds_msg.led()[i] = ros2_msg.led[i];
    }
    
    // Fan (both are uint8[2]/uint8[4] arrays depending on message version)
    for (int i = 0; i < 2; ++i) {
        dds_msg.fan()[i] = ros2_msg.fan[i];
    }
    
    dds_msg.gpio() = ros2_msg.gpio;
    dds_msg.reserve() = ros2_msg.reserve;
    
    // CRC will be calculated by caller
}

} // namespace hardware_unitree_ros2
