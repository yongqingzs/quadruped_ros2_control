//
// Created for hardware_free_dog_sdk
//

#ifndef HARDWAREFREEDOGSDK_H
#define HARDWAREFREEDOGSDK_H

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <fdsc_utils/free_dog_sdk_h.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>

// Macro definition for using external IMU
// #define USE_EXTERNAL_IMU

class HardwareFreeDogSdk final : public hardware_interface::SystemInterface {
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;

protected:
    // Joint command interfaces
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    // Joint state interfaces
    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    // Sensor interfaces
    std::vector<double> imu_states_;     // orientation(4) + angular_velocity(3) + linear_acceleration(3)
    std::vector<double> foot_force_;     // 4 foot force sensors
    std::vector<double> high_states_;    // high-level state data

    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}}
    };

    // Free Dog SDK communication
    std::shared_ptr<FDSC::UnitreeConnection> udp_connection_;
    FDSC::lowCmd low_cmd_;
    FDSC::lowState low_state_;
    FDSC::highState high_state_;

    // Configuration parameters
    std::string connection_settings_ = "LOW_WIRED_DEFAULTS";
    bool show_foot_force_ = false;
    bool use_high_level_ = false;
    double power_limit_ = 0.0;

    // Communication thread and synchronization
    std::thread output_thread_;
    std::mutex data_mutex_;
    std::atomic<bool> stop_output_thread_{false};
    std::atomic<bool> stop_node_spin_thread_{false};

    // ROS2 Publishers
    rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr low_state_publisher_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_publisher_;

    // ROS2 node for publishers and subscribers
    rclcpp::Node::SharedPtr node_;
    std::thread node_spin_thread_;

// #ifdef USE_EXTERNAL_IMU
    // External IMU subscriber
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_;
    std::mutex imu_mutex_;
// #endif

    // Internal methods
    void initLowCmd();
    void outputValues();
    void publishLowState();
    void publishLowCmd();
    void updateLowStateData();
    void updateHighStateData();
    void sendLowCmd();
    
    // Data conversion helpers
    void convertJointDataFromFDSC();
    void convertJointDataToFDSC();
    void convertImuData();
    void convertFootForceData();

    // IMU callback
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

#endif //HARDWAREFREEDOGSDK_H