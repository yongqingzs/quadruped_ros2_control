//
// Created by tlab-uav on 24-9-6.
//

#ifndef STATEPASSIVE_H
#define STATEPASSIVE_H
#include "FSMState.h"
#include <vector>

class StatePassive final : public FSMState
{
public:
    explicit StatePassive(CtrlInterfaces& ctrl_interfaces);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    std::vector<double> init_pose_;
    std::vector<double> stand_down_pose_;
    double transition_time_;
    double running_time_;
    bool transitioning_;
};


#endif //STATEPASSIVE_H
