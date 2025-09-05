//
// DDS-ROS2 Bridge Node Main Entry Point
//

#include <memory>
#include <csignal>
#include "rclcpp/rclcpp.hpp"
#include "hardware_unitree_ros2/dds_ros2_bridge_node.h"

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
