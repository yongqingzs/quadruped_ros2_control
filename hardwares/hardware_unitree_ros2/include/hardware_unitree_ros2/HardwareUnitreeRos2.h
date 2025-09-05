//
// Created for hardware_unitree_ros2 package
//

#ifndef HARDWARE_UNITREE_ROS2_H
#define HARDWARE_UNITREE_ROS2_H

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

class HardwareUnitreeRos2 : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(HardwareUnitreeRos2)

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // ROS2 Communication Bridge
    class Ros2CommunicationBridge;
    std::unique_ptr<Ros2CommunicationBridge> ros2_bridge_;

    // joint command
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    // joint state
    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    // sensor state
    std::vector<double> imu_states_;
    std::vector<double> foot_force_;
    std::vector<double> high_states_;

    std::map<std::string, std::vector<std::string>> joint_interfaces;

    void initLowCmd();
    void initializeRos2Bridge();
    void shutdownRos2Bridge();

    // Message handling
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void highStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);

    unitree_go::msg::LowCmd low_cmd_{}; // default init
    unitree_go::msg::LowState low_state_{}; // default init
    unitree_go::msg::SportModeState high_state_{}; // default init

    bool show_foot_force_ = false;
    
    // Safety and error handling
    std::atomic<bool> communication_active_{false};
    std::chrono::steady_clock::time_point last_state_time_;
    static constexpr double STATE_TIMEOUT_SEC = 1.0;

    // Thread safety
    mutable std::mutex state_mutex_;
    mutable std::mutex command_mutex_;
};

// Forward declaration for the bridge implementation
class HardwareUnitreeRos2::Ros2CommunicationBridge
{
public:
    Ros2CommunicationBridge();
    ~Ros2CommunicationBridge();
    
    bool initialize();
    void shutdown();
    
    // Publisher/Subscriber management
    void publishLowCmd(const unitree_go::msg::LowCmd& cmd);
    void setStateCallback(std::function<void(const unitree_go::msg::LowState::SharedPtr)> callback);
    void setHighStateCallback(std::function<void(const unitree_go::msg::SportModeState::SharedPtr)> callback);
    
    bool isActive() const { return active_.load(); }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_publisher_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_subscriber_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr high_state_subscriber_;
    
    std::unique_ptr<std::thread> spin_thread_;
    std::atomic<bool> active_{false};
    
    void spinThread();
};

#endif //HARDWARE_UNITREE_ROS2_H
