#ifndef RADIO_LINK_INPUT_H
#define RADIO_LINK_INPUT_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>

namespace radio_link_input {

class RadioLinkInput : public rclcpp::Node {
public:
    RadioLinkInput();
    ~RadioLinkInput();

private:
    // ROS2 publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr inputs_publisher_;

    // Serial communication
    int serial_fd_;
    std::string serial_port_;
    int baud_rate_;
    std::thread serial_thread_;
    std::atomic<bool> running_;
    std::mutex data_mutex_;

    // SBUS data
    static const int SBUS_FRAME_SIZE = 25;
    static const int SBUS_NUM_CHANNELS = 16;
    uint8_t sbus_buffer_[SBUS_FRAME_SIZE];
    uint16_t channels_[SBUS_NUM_CHANNELS];
    bool failsafe_;
    bool frame_lost_;

    // State machine
    enum class RobotState {
        RUN = 0,
        STOP = 1,
        STAND = 2,
        TROT = 3,
    };
    RobotState current_state_;

    // Channel mappings (configurable)
    struct ChannelMapping {
        int lx_channel;
        int ly_channel;
        int rx_channel;
        int ry_channel;
        int mode_0;
        int mode_1;
        int mode_2;
    };
    ChannelMapping channel_mapping_;


    // State tracking for determineRobotState
    float last_normal_0_ = 0.0f;
    float last_normal_1_ = 0.0f;
    RobotState last_state_ = RobotState::STOP;

    // Methods
    bool setupSerial();
    void serialReadLoop();
    bool parseSBUSFrame(const uint8_t* data, uint16_t* channels);
    void processChannels();
    void publishJoyMessage();
    void publishInputsMessage();
    float normalizeChannelValue(uint16_t value);
    RobotState determineRobotState(uint16_t mode_0, uint16_t mode_1, uint16_t mode_2);
};

} // namespace radio_link_input

#endif // RADIO_LINK_INPUT_H