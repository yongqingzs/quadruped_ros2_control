//
// Created by tlab-uav on 25-2-27.
//

#include "controller_common/FSM/StatePassive.h"
#include <cmath>

StatePassive::StatePassive(CtrlInterfaces& ctrl_interfaces) : FSMState(
    FSMStateName::PASSIVE, "passive", ctrl_interfaces),
    // stand_down_pose_({0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375}),
    stand_down_pose_({0.28, 1.19, -2.787, -0.28, 1.19, -2.787, 0.28, 1.19, -2.787, -0.28, 1.19, -2.787}),
    transition_time_(2.0),
    running_time_(0.0),
    transitioning_(false)
{
}

void StatePassive::enter()
{
    // Record initial joint positions
    init_pose_.clear();
    for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
    {
        auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
        if (value)
        {
            init_pose_.push_back(*value);
        }
        else
        {
            init_pose_.push_back(0.0); // Fallback
        }
    }
    running_time_ = 0.0;
    transitioning_ = true;
}

void StatePassive::run(const rclcpp::Time&/*time*/, const rclcpp::Duration& period)
{
    if (transitioning_)
    {
        running_time_ += period.seconds();
        double phase = tanh(running_time_ / transition_time_);

        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size(); ++i)
        {
            double target_pos = phase * stand_down_pose_[i] + (1 - phase) * init_pose_[i];
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(target_pos);
            std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(30.0);
            std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(1.5);
            std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
        }

        if (phase >= 0.99)
        {
            transitioning_ = false;
            // Set final commands after transition
            for (auto i : ctrl_interfaces_.joint_torque_command_interface_)
            {
                std::ignore = i.get().set_value(0.0);
            }
            for (auto i : ctrl_interfaces_.joint_position_command_interface_)
            {
                std::ignore = i.get().set_value(0.0);
            }
            for (auto i : ctrl_interfaces_.joint_velocity_command_interface_)
            {
                std::ignore = i.get().set_value(0.0);
            }
            for (auto i : ctrl_interfaces_.joint_kp_command_interface_)
            {
                std::ignore = i.get().set_value(0.0);
            }
            for (auto i : ctrl_interfaces_.joint_kd_command_interface_)
            {
                std::ignore = i.get().set_value(1.0);
            }
            ctrl_interfaces_.control_inputs_.command = 0;
        }
    }
}

void StatePassive::exit()
{
}

FSMStateName StatePassive::checkChange()
{
    if (ctrl_interfaces_.control_inputs_.command == 2)
    {
        return FSMStateName::FIXEDDOWN;
    }
    return FSMStateName::PASSIVE;
}
