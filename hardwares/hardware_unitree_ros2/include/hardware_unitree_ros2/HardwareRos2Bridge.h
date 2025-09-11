//
// DDS-ROS2 Bridge Node for hardware_unitree_ros2
//

#ifndef HARDWARE_UNITREE_ROS2_DDS_ROS2_BRIDGE_NODE_H
#define HARDWARE_UNITREE_ROS2_DDS_ROS2_BRIDGE_NODE_H

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "control_input_msgs/msg/inputs.hpp"

// DDS related includes (similar to hardware_unitree_sdk2)
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "hardware_unitree_ros2/crc32.h"

#include <fstream>
#include <deque>
#include <string>
#include <iomanip>
#include <sstream>

namespace hardware_unitree_ros2 {

class DdsRos2BridgeNode : public rclcpp::Node
{
public:
    explicit DdsRos2BridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DdsRos2BridgeNode();

    bool initialize();
    void shutdown();

private:
    // DDS publishers and subscribers (to communicate with unitree_mujoco or hardware)
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> dds_low_cmd_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> dds_low_state_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> dds_high_state_subscriber_;

    // ROS2 publishers and subscribers (to communicate with hardware_unitree_ros2)
    rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr ros2_low_state_publisher_;
    rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr ros2_high_state_publisher_;
    rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr ros2_low_cmd_subscriber_;
    
    // Control input subscriber for data logging
    rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_inputs_subscriber_;

    // Message handlers
    void ddsLowStateHandler(const void* message);
    void ddsHighStateHandler(const void* message);
    void ros2LowCmdHandler(const unitree_go::msg::LowCmd::SharedPtr msg);
    void controlInputsHandler(const control_input_msgs::msg::Inputs::SharedPtr msg);

    // Data logging methods
    void startDataLogging();
    void stopDataLogging();
    void recordData(const unitree_go::msg::LowState& low_state, const unitree_go::msg::LowCmd& low_cmd);
    void saveLoggedDataToCsv();
    void call_visualization_script(const std::string& abs_csv);

    // Message converters
    void convertDdsToRos2LowState(const unitree_go::msg::dds_::LowState_& dds_msg, 
                                  unitree_go::msg::LowState& ros2_msg);
    void convertDdsToRos2SportModeState(const unitree_go::msg::dds_::SportModeState_& dds_msg, 
                                        unitree_go::msg::SportModeState& ros2_msg);
    void convertRos2ToDdsLowCmd(const unitree_go::msg::LowCmd& ros2_msg, 
                                unitree_go::msg::dds_::LowCmd_& dds_msg);

    // Parameters
    std::string network_interface_;
    int domain_;
    bool debug_output_;
    bool log_flag_;

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
    unitree_go::msg::LowCmd last_low_cmd_;  // Store last command for logging
    rclcpp::TimerBase::SharedPtr logging_timer_;
    int logging_duration_; // seconds
    rclcpp::Time last_record_time_;
    std::deque<LoggedData> temp_buffer_;

    // DDS topic names (matching hardware_unitree_sdk2)
    static constexpr const char* TOPIC_LOWCMD = "rt/lowcmd";
    static constexpr const char* TOPIC_LOWSTATE = "rt/lowstate";
    static constexpr const char* TOPIC_HIGHSTATE = "rt/sportmodestate";

    // ROS2 topic names
    static constexpr const char* ROS2_TOPIC_LOWCMD = "unitree_go/low_cmd";
    static constexpr const char* ROS2_TOPIC_LOWSTATE = "unitree_go/low_state";
    static constexpr const char* ROS2_TOPIC_HIGHSTATE = "unitree_go/high_state";
    static constexpr const char* ROS2_TOPIC_CONTROL_INPUTS = "control_inputs";
};

} // namespace hardware_unitree_ros2

#endif // HARDWARE_UNITREE_ROS2_DDS_ROS2_BRIDGE_NODE_H
