//
// DDS-ROS2 Bridge Node Implementation for hardware_unitree
//

#include "hardware_unitree_ros2/HardwareRos2Bridge.h"
#include <unitree/robot/channel/channel_factory.hpp>
#include <csignal>

using namespace unitree::robot;

namespace hardware_unitree_ros2 {

DdsRos2BridgeNode::DdsRos2BridgeNode(const rclcpp::NodeOptions& options)
    : Node("dds_ros2_bridge_node", options)
{
    // Declare parameters
    declare_parameter("network_interface", "lo");
    declare_parameter("domain", 1);
    declare_parameter("debug_output", false);
    declare_parameter("log_flag", false);
    declare_parameter("logging_duration", 3);  // New parameter for logging duration

    // Get parameters
    network_interface_ = get_parameter("network_interface").as_string();
    domain_ = get_parameter("domain").as_int();
    debug_output_ = get_parameter("debug_output").as_bool();
    log_flag_ = get_parameter("log_flag").as_bool();
    logging_duration_ = get_parameter("logging_duration").as_int();  // Get logging duration

    RCLCPP_INFO(get_logger(), "DDS-ROS2 Bridge Node starting with interface: %s, domain: %d, log_flag: %s", 
                network_interface_.c_str(), domain_, log_flag_ ? "true" : "false");
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

        // Subscribe to control inputs for data logging (if enabled)
        if (log_flag_) {
            control_inputs_subscriber_ = create_subscription<control_input_msgs::msg::Inputs>(
                "control_input", qos,
                [this](const control_input_msgs::msg::Inputs::SharedPtr msg) { controlInputsHandler(msg); });
            
            RCLCPP_INFO(get_logger(), "Data logging enabled - subscribed to control inputs");
        }

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
    
    // Stop any ongoing logging
    if (is_logging_) {
        stopDataLogging();
    }
    
    // Reset DDS publishers and subscribers
    dds_low_cmd_publisher_.reset();
    dds_low_state_subscriber_.reset();
    dds_high_state_subscriber_.reset();
    
    // Reset ROS2 publishers and subscribers
    ros2_low_state_publisher_.reset();
    ros2_high_state_publisher_.reset();
    ros2_low_cmd_subscriber_.reset();
    control_inputs_subscriber_.reset();
    
    // Reset logging timer
    if (logging_timer_) {
        logging_timer_.reset();
    }
    
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
        
        // Record data if logging is active
        if (is_logging_ && log_flag_) {
            recordData(*ros2_low_state, last_low_cmd_);
        }
        
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
        
        // Store the latest low command for logging
        last_low_cmd_ = *msg;
        
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

void DdsRos2BridgeNode::controlInputsHandler(const control_input_msgs::msg::Inputs::SharedPtr msg)
{
    if (!log_flag_) return;
    if (msg->command == 0) return;

    int32_t current_command = msg->command;
    int32_t expected_last_command = last_command_.load();
    std::cout << "Current command: " << current_command << ", Last command: " << expected_last_command << std::endl;

    if (expected_last_command <= 1 && current_command == 1) {
        // First command received, just store it
        last_command_.store(current_command);
        return;
    }

    // Check if command has changed
    if (expected_last_command != current_command) {
        last_command_.store(current_command);

        RCLCPP_INFO(get_logger(), "Control command changed from %d to %d - starting %d seconds data logging", 
                    expected_last_command, current_command, logging_duration_);

        startDataLogging();
    }
}

void DdsRos2BridgeNode::startDataLogging()
{
    std::lock_guard<std::mutex> lock(logging_mutex_);
    if (is_logging_) {
        if (logging_timer_) {
            logging_timer_->cancel();
        }
    } else {
        logged_data_.clear();
        is_logging_ = true;
    }
    // 使用 node clock 保证时间源一致
    auto node_clock = this->get_clock();
    logging_start_time_ = node_clock->now();
    last_record_time_ = node_clock->now();
    logging_timer_ = create_wall_timer(
        std::chrono::seconds(logging_duration_),
        [this]() { stopDataLogging(); });
    RCLCPP_INFO(get_logger(), "Started data logging for %d seconds", logging_duration_);
}

void DdsRos2BridgeNode::stopDataLogging()
{
    std::lock_guard<std::mutex> lock(logging_mutex_);
    
    if (!is_logging_) return;
    
    is_logging_ = false;
    
    if (logging_timer_) {
        logging_timer_->cancel();
        logging_timer_.reset();
    }
    
    RCLCPP_INFO(get_logger(), "Stopped data logging. Recorded %zu data points", logged_data_.size());
    
    // Save logged data to CSV
    saveLoggedDataToCsv();
}

void DdsRos2BridgeNode::recordData(const unitree_go::msg::LowState& low_state, const unitree_go::msg::LowCmd& low_cmd)
{
    if (!is_logging_) return;
    std::lock_guard<std::mutex> lock(logging_mutex_);
    auto node_clock = this->get_clock();
    // Add data to a temporary buffer
    LoggedData data;
    data.timestamp = (node_clock->now() - logging_start_time_).seconds();
    data.low_state = low_state;
    data.low_cmd = low_cmd;
    temp_buffer_.push_back(data);
    // Only save to logged_data_ every 10ms (100Hz)
    auto now_time = node_clock->now();
    if ((now_time - last_record_time_).seconds() >= 0.01) {
        logged_data_.insert(logged_data_.end(), temp_buffer_.begin(), temp_buffer_.end());
        temp_buffer_.clear();
        last_record_time_ = now_time;
    }

    // Limit logged_data_ size to prevent excessive memory usage
    // if (logged_data_.size() > 1000) {
    //     logged_data_.pop_front();
    // }
}

void DdsRos2BridgeNode::saveLoggedDataToCsv()
{
    if (logged_data_.empty()) {
        RCLCPP_WARN(get_logger(), "No data to save");
        return;
    }
    
    // Generate filename with timestamp
    auto now_time = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now_time);
    std::stringstream ss;
    ss << "data/logged_data_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
    std::string filename = ss.str();
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open file %s for writing", filename.c_str());
        return;
    }
    
    // Write CSV header
    file << "timestamp,";
    
    // Low state headers
    file << "imu_quat_w,imu_quat_x,imu_quat_y,imu_quat_z,";
    file << "imu_gyro_x,imu_gyro_y,imu_gyro_z,";
    file << "imu_accel_x,imu_accel_y,imu_accel_z,";
    file << "imu_rpy_r,imu_rpy_p,imu_rpy_y,";
    file << "imu_temperature,";
    
    // Motor state headers (20 motors)
    for (int i = 0; i < 20; ++i) {
        file << "motor_" << i << "_mode,motor_" << i << "_q,motor_" << i << "_dq,motor_" << i << "_ddq,";
        file << "motor_" << i << "_tau,motor_" << i << "_temperature,";
    }
    
    // Motor command headers (20 motors)
    for (int i = 0; i < 20; ++i) {
        file << "cmd_motor_" << i << "_mode,cmd_motor_" << i << "_q,cmd_motor_" << i << "_dq,";
        file << "cmd_motor_" << i << "_tau,cmd_motor_" << i << "_kp,cmd_motor_" << i << "_kd,";
    }
    
    file << "\n";
    
    // Write data
    for (const auto& data : logged_data_) {
        file << std::fixed << std::setprecision(6) << data.timestamp << ",";
        
        // IMU data
        const auto& imu = data.low_state.imu_state;
        file << imu.quaternion[0] << "," << imu.quaternion[1] << "," << imu.quaternion[2] << "," << imu.quaternion[3] << ",";
        file << imu.gyroscope[0] << "," << imu.gyroscope[1] << "," << imu.gyroscope[2] << ",";
        file << imu.accelerometer[0] << "," << imu.accelerometer[1] << "," << imu.accelerometer[2] << ",";
        file << imu.rpy[0] << "," << imu.rpy[1] << "," << imu.rpy[2] << ",";
        file << imu.temperature << ",";
        
        // Motor states
        for (int i = 0; i < 20; ++i) {
            const auto& motor = data.low_state.motor_state[i];
            file << motor.mode << "," << motor.q << "," << motor.dq << "," << motor.ddq << ",";
            file << motor.tau_est << "," << motor.temperature << ",";
        }
        
        // Motor commands
        for (int i = 0; i < 20; ++i) {
            const auto& cmd = data.low_cmd.motor_cmd[i];
            file << cmd.mode << "," << cmd.q << "," << cmd.dq << ",";
            file << cmd.tau << "," << cmd.kp << "," << cmd.kd << ",";
        }
        
        file << "\n";
    }
    
    file.close();
    
    RCLCPP_INFO(get_logger(), "Saved %zu data points to %s", logged_data_.size(), filename.c_str());
    
    // Clear logged data to free memory
    logged_data_.clear();
}

} // namespace hardware_unitree_ros2

std::shared_ptr<hardware_unitree_ros2::DdsRos2BridgeNode> g_bridge_node = nullptr;

void signalHandler(int signal) {
    if (g_bridge_node) {
        RCLCPP_INFO(g_bridge_node->get_logger(), "Received signal %d, shutting down...", signal);
        g_bridge_node->shutdown();
        rclcpp::shutdown();
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    // Install signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    try {
        // Create bridge node
        auto options = rclcpp::NodeOptions();
        g_bridge_node = std::make_shared<hardware_unitree_ros2::DdsRos2BridgeNode>(options);
        
        // Initialize the bridge
        if (!g_bridge_node->initialize()) {
            RCLCPP_ERROR(g_bridge_node->get_logger(), "Failed to initialize DDS-ROS2 bridge");
            return 1;
        }
        
        RCLCPP_INFO(g_bridge_node->get_logger(), "DDS-ROS2 Bridge Node is running...");
        RCLCPP_INFO(g_bridge_node->get_logger(), "Bridge Topics:");
        RCLCPP_INFO(g_bridge_node->get_logger(), "  DDS -> ROS2: rt/lowstate -> unitree_go/low_state");
        RCLCPP_INFO(g_bridge_node->get_logger(), "  DDS -> ROS2: rt/sportmodestate -> unitree_go/high_state");
        RCLCPP_INFO(g_bridge_node->get_logger(), "  ROS2 -> DDS: unitree_go/low_cmd -> rt/lowcmd");
        
        // Spin the node
        rclcpp::spin(g_bridge_node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    }
    
    if (g_bridge_node) {
        g_bridge_node->shutdown();
    }
    
    rclcpp::shutdown();
    return 0;
}
