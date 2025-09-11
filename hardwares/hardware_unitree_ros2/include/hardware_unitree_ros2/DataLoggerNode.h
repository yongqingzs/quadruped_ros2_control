//
// Data Logger and Visualization Node for hardware_unitree_ros2
//

#ifndef HARDWARE_UNITREE_ROS2_DATA_LOGGER_NODE_H
#define HARDWARE_UNITREE_ROS2_DATA_LOGGER_NODE_H

#include <memory>
#include <atomic>
#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "control_input_msgs/msg/inputs.hpp"

#include <fstream>
#include <deque>
#include <string>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <array>
#include <cstdio>

namespace hardware_unitree_ros2 {

class DataLoggerNode : public rclcpp::Node
{
public:
    explicit DataLoggerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DataLoggerNode();

    bool initialize();
    void shutdown();

private:
    // ROS2 subscribers
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_subscriber_;
    rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr low_cmd_subscriber_;
    rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_inputs_subscriber_;

    // Message handlers
    void lowStateHandler(const unitree_go::msg::LowState::SharedPtr msg);
    void lowCmdHandler(const unitree_go::msg::LowCmd::SharedPtr msg);
    void controlInputsHandler(const control_input_msgs::msg::Inputs::SharedPtr msg);

    // Data logging methods
    void startDataLogging();
    void stopDataLogging();
    void recordData(const unitree_go::msg::LowState& low_state, const unitree_go::msg::LowCmd& low_cmd);
    void saveLoggedDataToCsv();
    void call_visualization_script(const std::string& abs_csv);

    // Parameters
    bool log_flag_;
    int logging_duration_; // seconds

    // State management
    std::atomic<bool> active_{false};
    mutable std::mutex state_mutex_;
    
    // Data logging state
    std::atomic<bool> is_logging_{false};
    std::atomic<int32_t> last_command_{-1};
    rclcpp::Time logging_start_time_;
    mutable std::mutex logging_mutex_;
    
    struct LoggedData {
        double timestamp;
        unitree_go::msg::LowState low_state;
        unitree_go::msg::LowCmd low_cmd;
    };
    std::deque<LoggedData> logged_data_;
    unitree_go::msg::LowState last_low_state_;  // Store last state for logging
    unitree_go::msg::LowCmd last_low_cmd_;      // Store last command for logging
    rclcpp::TimerBase::SharedPtr logging_timer_;
    rclcpp::Time last_record_time_;
    std::deque<LoggedData> temp_buffer_;

    // ROS2 topic names
    static constexpr const char* ROS2_TOPIC_LOWCMD = "unitree_go/low_cmd";
    static constexpr const char* ROS2_TOPIC_LOWSTATE = "unitree_go/low_state";
    static constexpr const char* ROS2_TOPIC_CONTROL_INPUTS = "control_input";
};

} // namespace hardware_unitree_ros2

#endif // HARDWARE_UNITREE_ROS2_DATA_LOGGER_NODE_H
