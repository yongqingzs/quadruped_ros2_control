//
// Data Logger and Visualization Node Implementation for hardware_unitree_ros2
//

#include "hardware_unitree_ros2/DataLoggerNode.h"
#include <csignal>

namespace hardware_unitree_ros2 {

DataLoggerNode::DataLoggerNode(const rclcpp::NodeOptions& options)
    : Node("data_logger_node", options)
{
    // Declare parameters
    declare_parameter("log_flag", true);
    declare_parameter("logging_duration", 3);  // seconds

    // Get parameters
    log_flag_ = get_parameter("log_flag").as_bool();
    logging_duration_ = get_parameter("logging_duration").as_int();

    RCLCPP_INFO(get_logger(), "Data Logger Node starting with log_flag: %s, logging_duration: %d seconds", 
                log_flag_ ? "true" : "false", logging_duration_);
}

DataLoggerNode::~DataLoggerNode()
{
    shutdown();
}

bool DataLoggerNode::initialize()
{
    try {
        // Create ROS2 subscribers
        auto qos = rclcpp::QoS(10).reliable();
        
        low_state_subscriber_ = create_subscription<unitree_go::msg::LowState>(
            ROS2_TOPIC_LOWSTATE, qos,
            [this](const unitree_go::msg::LowState::SharedPtr msg) { lowStateHandler(msg); });
            
        low_cmd_subscriber_ = create_subscription<unitree_go::msg::LowCmd>(
            ROS2_TOPIC_LOWCMD, qos,
            [this](const unitree_go::msg::LowCmd::SharedPtr msg) { lowCmdHandler(msg); });

        // Subscribe to control inputs for data logging trigger
        if (log_flag_) {
            control_inputs_subscriber_ = create_subscription<control_input_msgs::msg::Inputs>(
                ROS2_TOPIC_CONTROL_INPUTS, qos,
                [this](const control_input_msgs::msg::Inputs::SharedPtr msg) { controlInputsHandler(msg); });
            
            RCLCPP_INFO(get_logger(), "Data logging enabled - subscribed to control inputs");
        }

        active_ = true;
        
        RCLCPP_INFO(get_logger(), "Data Logger Node initialized successfully");
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize Data Logger Node: %s", e.what());
        return false;
    }
}

void DataLoggerNode::shutdown()
{
    active_ = false;
    
    // Stop any ongoing logging
    if (is_logging_) {
        stopDataLogging();
    }
    
    // Reset ROS2 subscribers
    low_state_subscriber_.reset();
    low_cmd_subscriber_.reset();
    control_inputs_subscriber_.reset();
    
    // Reset logging timer
    if (logging_timer_) {
        logging_timer_.reset();
    }
    
    RCLCPP_INFO(get_logger(), "Data Logger Node shutdown complete");
}

void DataLoggerNode::lowStateHandler(const unitree_go::msg::LowState::SharedPtr msg)
{
    if (!active_) return;
    
    try {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Store the latest low state for logging
        last_low_state_ = *msg;
        
        // Record data if logging is active
        if (is_logging_ && log_flag_) {
            recordData(*msg, last_low_cmd_);
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in LowState handler: %s", e.what());
    }
}

void DataLoggerNode::lowCmdHandler(const unitree_go::msg::LowCmd::SharedPtr msg)
{
    if (!active_) return;
    
    try {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Store the latest low command for logging
        last_low_cmd_ = *msg;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in LowCmd handler: %s", e.what());
    }
}

void DataLoggerNode::controlInputsHandler(const control_input_msgs::msg::Inputs::SharedPtr msg)
{
    if (!log_flag_) return;
    if (msg->command == 0) return;

    int32_t current_command = msg->command;
    int32_t expected_last_command = last_command_.load();
    
    RCLCPP_DEBUG(get_logger(), "Current command: %d, Last command: %d", current_command, expected_last_command);

    if (expected_last_command <= 1 && current_command == 1) {
        // First command received, just store it
        last_command_.store(current_command);
        return;
    }

    // Check if command has changed
    // if (expected_last_command != current_command) {
        last_command_.store(current_command);

        RCLCPP_INFO(get_logger(), "Control command changed from %d to %d - starting %d seconds data logging", 
                    expected_last_command, current_command, logging_duration_);

        startDataLogging();
    // }
}

void DataLoggerNode::startDataLogging()
{
    std::lock_guard<std::mutex> lock(logging_mutex_);
    if (is_logging_) {
        if (logging_timer_) {
            logging_timer_->cancel();
        }
    }
    // Always reset for a new logging session to ensure timestamps start from 0
    logged_data_.clear();
    is_logging_ = true;
    
    // 使用 node clock 保证时间源一致
    auto node_clock = this->get_clock();
    logging_start_time_ = node_clock->now();
    last_record_time_ = node_clock->now();
    logging_timer_ = create_wall_timer(
        std::chrono::seconds(logging_duration_),
        [this]() { stopDataLogging(); });
    
    RCLCPP_INFO(get_logger(), "Started data logging for %d seconds", logging_duration_);
}

void DataLoggerNode::stopDataLogging()
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

void DataLoggerNode::recordData(const unitree_go::msg::LowState& low_state, const unitree_go::msg::LowCmd& low_cmd)
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
}

void DataLoggerNode::saveLoggedDataToCsv()
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
        file << static_cast<int>(imu.temperature) << ","; // 修正

        // Motor states
        for (int i = 0; i < 20; ++i) {
            const auto& motor = data.low_state.motor_state[i];
            file << static_cast<int>(motor.mode) << "," // 修正
                << motor.q << "," << motor.dq << "," << motor.ddq << ","
                << motor.tau_est << "," 
                << static_cast<int>(motor.temperature) << ","; // 修正
        }

        // Motor commands
        for (int i = 0; i < 20; ++i) {
            const auto& cmd = data.low_cmd.motor_cmd[i];
            file << static_cast<int>(cmd.mode) << "," // 修正
                << cmd.q << "," << cmd.dq << ","
                << cmd.tau << "," << cmd.kp << "," << cmd.kd << ",";
        }
                
        file << "\n";
    }
    
    file.close();
    RCLCPP_INFO(get_logger(), "Saved %zu data points to %s", logged_data_.size(), filename.c_str());

    std::string abs_csv = (std::filesystem::current_path() / filename).string();
    call_visualization_script(abs_csv);
    
    // Clear logged data to free memory
    logged_data_.clear();
}

void DataLoggerNode::call_visualization_script(const std::string& abs_csv)
{
    std::string pkg_dir;
    {
        std::array<char, 256> buffer{};
        std::string cmd = "ros2 pkg prefix hardware_unitree_ros2 2>/dev/null";
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
        if (pipe) {
            if (fgets(buffer.data(), buffer.size(), pipe.get())) {
                pkg_dir = std::string(buffer.data());
                pkg_dir.erase(pkg_dir.find_last_not_of("\n\r") + 1);
            }
        }
        if (pkg_dir.empty()) {
            RCLCPP_WARN(get_logger(), "Failed to resolve package directory using ros2 pkg prefix, fallback to current directory");
            pkg_dir = ".";
        }
    }
    std::string script_path = pkg_dir + "/share/hardware_unitree_ros2/scripts/visualize_logged_data.py";

    std::string home = std::getenv("HOME") ? std::getenv("HOME") : "";
    std::string venv_dir = home + "/venvs/motor";
    std::string python_bin = venv_dir + "/bin/python3";
    std::string activate_script = venv_dir + "/bin/activate";

    bool venv_exists = std::filesystem::exists(python_bin);
    if (!venv_exists) {
        std::string mkvenv = "python3 -m venv '" + venv_dir + "'";
        int ret1 = std::system(mkvenv.c_str());
        if (ret1 != 0) {
            RCLCPP_WARN(get_logger(), "Failed to create venv at %s", venv_dir.c_str());
            return;
        }
        std::string pip_install = "'" + python_bin + "' -m pip install --upgrade pip && '" + python_bin + "' -m pip install numpy matplotlib pandas";
        int ret2 = std::system(pip_install.c_str());
        if (ret2 != 0) {
            RCLCPP_WARN(get_logger(), "Failed to install python packages in venv");
            return;
        }
    }

    std::string cmd = "bash -c \"source '" + activate_script + "' && '" + python_bin + "' '" + script_path + "' '" + abs_csv + "' --save\" &";
    int ret = std::system(cmd.c_str());
    if (ret != 0) {
        RCLCPP_WARN(get_logger(), "Failed to call visualize_logged_data.py, return code: %d", ret);
    }
}

} // namespace hardware_unitree_ros2

std::shared_ptr<hardware_unitree_ros2::DataLoggerNode> g_data_logger_node = nullptr;

void signalHandler(int signal) {
    if (g_data_logger_node) {
        RCLCPP_INFO(g_data_logger_node->get_logger(), "Received signal %d, shutting down...", signal);
        g_data_logger_node->shutdown();
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
        // Create data logger node
        auto options = rclcpp::NodeOptions();
        g_data_logger_node = std::make_shared<hardware_unitree_ros2::DataLoggerNode>(options);
        
        // Initialize the data logger
        if (!g_data_logger_node->initialize()) {
            RCLCPP_ERROR(g_data_logger_node->get_logger(), "Failed to initialize Data Logger Node");
            return 1;
        }
        
        RCLCPP_INFO(g_data_logger_node->get_logger(), "Data Logger Node is running...");
        RCLCPP_INFO(g_data_logger_node->get_logger(), "Subscribed Topics:");
        RCLCPP_INFO(g_data_logger_node->get_logger(), "  - unitree_go/low_state");
        RCLCPP_INFO(g_data_logger_node->get_logger(), "  - unitree_go/low_cmd");
        RCLCPP_INFO(g_data_logger_node->get_logger(), "  - control_input");
        
        // Spin the node
        rclcpp::spin(g_data_logger_node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    }
    
    if (g_data_logger_node) {
        g_data_logger_node->shutdown();
    }
    
    rclcpp::shutdown();
    return 0;
}
