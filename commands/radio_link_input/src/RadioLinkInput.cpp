#include "radio_link_input/RadioLinkInput.h"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <sys/ioctl.h>     // For ioctl
// #include <asm/termbits.h>  // For termios2
#include <termios.h>       // For tcflush
#include <cmath>           // For std::abs

namespace radio_link_input {

#ifndef TCGETS2
#define TCGETS2     _IOR('T', 0x2A, struct termios2)
#define TCSETS2     _IOW('T', 0x2B, struct termios2)
#endif

#ifndef BOTHER
#define BOTHER 0x00001000
#endif

struct termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t c_line;
    cc_t c_cc[19];
    speed_t c_ispeed;
    speed_t c_ospeed;
};

RadioLinkInput::RadioLinkInput()
    : Node("radio_link_input"),
      serial_fd_(-1),
      serial_port_("/dev/ttyS0"),  // Default serial port
      baud_rate_(100000),            // SBUS baud rate
      running_(false),
      failsafe_(false),
      frame_lost_(false),
      current_state_(RobotState::STOP) {

    // Declare parameters
    this->declare_parameter<std::string>("serial_port", serial_port_);
    this->declare_parameter<int>("baud_rate", baud_rate_);
    this->declare_parameter<int>("lx_channel", 3);
    this->declare_parameter<int>("ly_channel", 2);
    this->declare_parameter<int>("rx_channel", 0);
    this->declare_parameter<int>("ry_channel", 1);
    this->declare_parameter<int>("mode_0", 9);
    this->declare_parameter<int>("mode_1", 6);
    this->declare_parameter<int>("mode_2", 4);

    // Get parameters
    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("lx_channel", channel_mapping_.lx_channel);
    this->get_parameter("ly_channel", channel_mapping_.ly_channel);
    this->get_parameter("rx_channel", channel_mapping_.rx_channel);
    this->get_parameter("ry_channel", channel_mapping_.ry_channel);
    this->get_parameter("mode_0", channel_mapping_.mode_0);
    this->get_parameter("mode_1", channel_mapping_.mode_1);
    this->get_parameter("mode_2", channel_mapping_.mode_2);

    // Create publishers
    // joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    inputs_publisher_ = this->create_publisher<control_input_msgs::msg::Inputs>("control_input", 10);

    // Setup serial communication
    if (setupSerial()) {
        running_ = true;
        serial_thread_ = std::thread(&RadioLinkInput::serialReadLoop, this);
        RCLCPP_INFO(this->get_logger(), "RadioLinkInput node started successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup serial communication");
    }
}

RadioLinkInput::~RadioLinkInput() {
    running_ = false;
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
    if (serial_fd_ >= 0) {
        close(serial_fd_);
    }
}

bool RadioLinkInput::setupSerial() {
    // 打开串口，使用 O_RDONLY | O_NOCTTY | O_NONBLOCK 标志使其以只读、非阻塞模式打开
    serial_fd_ = open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", strerror(errno));
        return false;
    }

    struct termios2 tio { };
    if (0 != ioctl(serial_fd_, TCGETS2, &tio)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get termios2 attributes");
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    /**
     * Setting serial port,8E2, non-blocking.100Kbps
     */
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
            | IXON);
    tio.c_iflag |= (INPCK | IGNPAR);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
    /**
     * use BOTHER to specify speed directly in c_[io]speed member
     */
    tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
    tio.c_ispeed = 100000;
    tio.c_ospeed = 100000;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (0 != ioctl(serial_fd_, TCSETS2, &tio)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set termios2 attributes");
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    tcflush(serial_fd_, TCIFLUSH);

    RCLCPP_INFO(this->get_logger(), "Serial port %s configured successfully", serial_port_.c_str());
    return true;
}

void RadioLinkInput::serialReadLoop() {
    uint8_t buffer[SBUS_FRAME_SIZE];
    int buffer_index = 0;
    bool in_frame = false;

    while (running_) {
        uint8_t byte;
        ssize_t n = read(serial_fd_, &byte, 1);

        if (n > 0) {
            // Look for start byte (0x0F)
            if (!in_frame && byte == 0x0F) {
                in_frame = true;
                buffer_index = 0;
                buffer[buffer_index++] = byte;
            } else if (in_frame) {
                buffer[buffer_index++] = byte;

                // Check if we have a complete frame
                if (buffer_index >= SBUS_FRAME_SIZE) {
                    in_frame = false;

                    // Process the frame
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    if (parseSBUSFrame(buffer, channels_)) {
                        processChannels();
                        // publishJoyMessage();
                        publishInputsMessage();
                    }
                }
            }
        } else if (n < 0 && errno != EAGAIN) {
            RCLCPP_WARN(this->get_logger(), "Serial read error: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Small delay to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

bool RadioLinkInput::parseSBUSFrame(const uint8_t* data, uint16_t* channels) {
    // SBUS frame format validation
    if (data[0] != 0x0F || data[24] != 0x00) {
        return false;
    }

    // Extract channel data (11 bits per channel, 16 channels)
    channels[0]  = ((data[1]    | data[2] <<8)                 & 0x07FF);
    channels[1]  = ((data[2]>>3 | data[3] <<5)                 & 0x07FF);
    channels[2]  = ((data[3]>>6 | data[4] <<2 | data[5]<<10)   & 0x07FF);
    channels[3]  = ((data[5]>>1 | data[6] <<7)                 & 0x07FF);
    channels[4]  = ((data[6]>>4 | data[7] <<4)                 & 0x07FF);
    channels[5]  = ((data[7]>>7 | data[8] <<1 | data[9]<<9)    & 0x07FF);
    channels[6]  = ((data[9]>>2 | data[10]<<6)                 & 0x07FF);
    channels[7]  = ((data[10]>>5| data[11]<<3)                 & 0x07FF);
    channels[8]  = ((data[12]   | data[13]<<8)                 & 0x07FF);
    channels[9]  = ((data[13]>>3| data[14]<<5)                 & 0x07FF);
    channels[10] = ((data[14]>>6| data[15]<<2 | data[16]<<10)  & 0x07FF);
    channels[11] = ((data[16]>>1| data[17]<<7)                 & 0x07FF);
    channels[12] = ((data[17]>>4| data[18]<<4)                 & 0x07FF);
    channels[13] = ((data[18]>>7| data[19]<<1 | data[20]<<9)   & 0x07FF);
    channels[14] = ((data[20]>>2| data[21]<<6)                 & 0x07FF);
    channels[15] = ((data[21]>>5| data[22]<<3)                 & 0x07FF);

    // Extract flags
    uint8_t flags = data[23];
    failsafe_ = (flags & 0x08) != 0;
    frame_lost_ = (flags & 0x04) != 0;

    return true;
}

void RadioLinkInput::processChannels() {
    // Determine robot state from mode channel
    uint16_t mode_0 = channels_[channel_mapping_.mode_0];
    uint16_t mode_1 = channels_[channel_mapping_.mode_1];
    uint16_t mode_2 = channels_[channel_mapping_.mode_2];
    current_state_ = determineRobotState(mode_0, mode_1, mode_2);

    // Update state tracking
    last_normal_0_ = normalizeChannelValue(mode_0);
    last_normal_1_ = normalizeChannelValue(mode_1);
    if (current_state_ != RobotState::RUN) last_state_ = current_state_;
}

void RadioLinkInput::publishJoyMessage() {
    auto joy_msg = sensor_msgs::msg::Joy();
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "radio_link";

    // Convert channels to joystick axes (normalized to -1.0 to 1.0)
    joy_msg.axes.resize(SBUS_NUM_CHANNELS);
    for (int i = 0; i < SBUS_NUM_CHANNELS; ++i) {
        joy_msg.axes[i] = normalizeChannelValue(channels_[i]);
    }

    // Set buttons based on state
    joy_msg.buttons.resize(5); // 5 possible states
    // joy_msg.buttons[static_cast<int>(RobotState::STOP)] = (current_state_ == RobotState::STOP) ? 1 : 0;
    // joy_msg.buttons[static_cast<int>(RobotState::STAND_UP)] = (current_state_ == RobotState::STAND_UP) ? 1 : 0;
    // joy_msg.buttons[static_cast<int>(RobotState::STAND_DOWN)] = (current_state_ == RobotState::STAND_DOWN) ? 1 : 0;
    // joy_msg.buttons[static_cast<int>(RobotState::WALK)] = (current_state_ == RobotState::WALK) ? 1 : 0;
    // joy_msg.buttons[static_cast<int>(RobotState::RUN)] = (current_state_ == RobotState::RUN) ? 1 : 0;

    joy_publisher_->publish(joy_msg);
}

void RadioLinkInput::publishInputsMessage() {
    auto inputs_msg = control_input_msgs::msg::Inputs();
    inputs_msg.command = static_cast<int32_t>(current_state_);

    // Map channels to control inputs
    if (inputs_msg.command == 0) {
        inputs_msg.lx = normalizeChannelValue(channels_[channel_mapping_.lx_channel]);
        inputs_msg.ly = normalizeChannelValue(channels_[channel_mapping_.ly_channel]);
        inputs_msg.rx = normalizeChannelValue(channels_[channel_mapping_.rx_channel]);
        inputs_msg.ry = normalizeChannelValue(channels_[channel_mapping_.ry_channel]);
    }
    inputs_publisher_->publish(inputs_msg);
}

float RadioLinkInput::normalizeChannelValue(uint16_t value) {
    // SBUS range: 172 (min) to 1811 (max), center at 992
    // const float SBUS_MIN = 172.0f;
    // const float SBUS_MAX = 1811.0f;
    // const float SBUS_CENTER = 992.0f;

    // // Clamp value to valid range
    // float clamped = std::max(SBUS_MIN, std::min(SBUS_MAX, static_cast<float>(value)));

    // // Normalize to -1.0 to 1.0
    // if (clamped < SBUS_CENTER) {
    //     return -1.0f + 2.0f * (clamped - SBUS_MIN) / (SBUS_CENTER - SBUS_MIN);
    // } else {
    //     return 2.0f * (clamped - SBUS_CENTER) / (SBUS_MAX - SBUS_CENTER);
    // }
    float clamped = (value - 307.0f) / (1693.0f - 307.0f) * 2.0f - 1.0f;
    if (abs(clamped) < 0.05f) clamped = 0.0f; // Deadzone

    return clamped;
}

RadioLinkInput::RobotState RadioLinkInput::determineRobotState(uint16_t mode_0, uint16_t mode_1, uint16_t mode_2) {
    // Map mode channel value to robot states
    float normal_0 = normalizeChannelValue(mode_0);
    float normal_1 = normalizeChannelValue(mode_1);
    float normal_2 = normalizeChannelValue(mode_2);

    // Calculate changes
    float delta_0 = normal_0 - last_normal_0_;
    float delta_1 = normal_1 - last_normal_1_;
    RobotState update_state = RobotState::RUN;

    // std::cout << "normal_0: " << normal_0 << ", last_normal_0_: " << last_normal_0_ << std::endl;
    // std::cout << "normal_1: " << normal_1 << ", last_normal_1_: " << last_normal_1_ << std::endl;

    auto safe_update = [&](RobotState& state, RobotState target) {
        if (target != last_state_) {
            state = target;
        }
    };

    if (delta_0 > 0.5f) {
        safe_update(update_state, RobotState::STAND);
    }
    else if (delta_0 < -0.5f) {
        safe_update(update_state, RobotState::STOP);
    }
    else if (delta_1 > 1.0f && last_state_ == RobotState::STAND) {
        safe_update(update_state, RobotState::TROT);
    }
    else if (delta_1 < -1.0f && last_state_ == RobotState::TROT) {
        safe_update(update_state, RobotState::STAND);
    }
    else {
    }

    if (static_cast<int>(update_state) != 0) std::cout << "update_state: " << static_cast<int>(update_state) << std::endl;
    return update_state;
}
} // namespace radio_link_input

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<radio_link_input::RadioLinkInput>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}