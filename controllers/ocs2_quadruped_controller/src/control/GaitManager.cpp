//
// Created by tlab-uav on 24-9-26.
//

#include <utility>

#include "ocs2_quadruped_controller/control/GaitManager.h"

#include <ocs2_core/misc/LoadData.h>

namespace ocs2::legged_robot
{
    GaitManager::GaitManager(CtrlInterfaces& ctrl_interfaces,
                             std::shared_ptr<GaitSchedule> gait_schedule_ptr,
                             bool enable_gait_adjust)
        : ctrl_interfaces_(ctrl_interfaces),
          gait_schedule_ptr_(std::move(gait_schedule_ptr)),
          adjust_active_(enable_gait_adjust),
          target_gait_({0.0, 1.0}, {STANCE}),
          previous_gait_({0.0, 1.0}, {STANCE})
    {
    }

    void GaitManager::preSolverRun(const scalar_t initTime, const scalar_t finalTime,
                                   const vector_t& /**currentState**/,
                                   const ReferenceManagerInterface& /**referenceManager**/)
    {
        getTargetGait();
        if (adjust_active_) adjustTargetGait();

        if (gait_updated_)
        {
            const auto timeHorizon = finalTime - initTime;
            gait_schedule_ptr_->insertModeSequenceTemplate(target_gait_, finalTime,
                                                           timeHorizon);
            gait_updated_ = false;
        }
    }

    void GaitManager::init(const std::string& gait_file)
    {
        gait_name_list_.clear();
        loadData::loadStdVector(gait_file, "list", gait_name_list_, verbose_);

        gait_list_.clear();
        for (const auto& name : gait_name_list_)
        {
            gait_list_.push_back(loadModeSequenceTemplate(gait_file, name, verbose_));
        }

        RCLCPP_INFO(rclcpp::get_logger("gait_manager"), "GaitManager is ready.");
    }

    void GaitManager::getTargetGait()
    {
        if (ctrl_interfaces_.control_inputs_.command == 0) return;
        if (ctrl_interfaces_.control_inputs_.command == last_command_) return;
        last_command_ = ctrl_interfaces_.control_inputs_.command;
        const int command = std::max(0, ctrl_interfaces_.control_inputs_.command - 2);
        target_gait_ = gait_list_[command];
        RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Switch to gait: %s",
                    gait_name_list_[command].c_str());
        gait_updated_ = true;
    }

    void GaitManager::adjustTargetGait()
    {
        const int delay_count = 200;
        int last_cmd = std::max(0, last_command_ - 2);

        // Check if robot should be moving
        bool is_moving = (std::abs(ctrl_interfaces_.control_inputs_.ly) >= 0.1 ||
                          std::abs(ctrl_interfaces_.control_inputs_.lx) >= 0.1 ||
                          std::abs(ctrl_interfaces_.control_inputs_.rx) >= 0.1 ||
                          std::abs(ctrl_interfaces_.control_inputs_.ry) >= 0.1);

        if (gait_updated_) 
        {
            pending_switch_to_stop_ = false;
            in_stop_mode_ = false;
        }

        // Determine action based on current state and conditions
        if (!is_moving) {
            if (last_cmd == 0) return;
            if (in_stop_mode_ && !gait_updated_) return;

            // Start pending switch to stop
            if (!pending_switch_to_stop_) 
            {
                pending_switch_to_stop_ = true;
                switch_delay_counter_ = 0;
                std::cout << "Starting switch to stop gait timer." << std::endl;
            }
            switch_delay_counter_++;
            // std::cout << "switch_delay_counter_: " << switch_delay_counter_ << std::endl;
            if (switch_delay_counter_ >= delay_count) 
            {
                previous_gait_ = target_gait_;  // Remember current gait
                target_gait_ = gait_list_[0];   // Switch to stance gait
                in_stop_mode_ = true;
                gait_updated_ = true;
                pending_switch_to_stop_ = false;
                RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Switch to stop gait due to low ly");
            }
        } 
        else if (is_moving) 
        {
            if (!in_stop_mode_) return;

            // Start pending restore to previous gait
            // if (!pending_restore_gait_) 
            // {
            //     pending_restore_gait_ = true;
            //     switch_delay_counter_ = 0;
            // }
            // switch_delay_counter_++;
            // if (switch_delay_counter_ >= delay_count) 
            // {
            //     target_gait_ = previous_gait_;  // Restore previous gait
            //     in_stop_mode_ = false;
            //     gait_updated_ = true;
            //     pending_restore_gait_ = false;
            //     RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Restore previous gait");
            // }
            target_gait_ = previous_gait_;  // Restore previous gait
            in_stop_mode_ = false;
            gait_updated_ = true;
            pending_restore_gait_ = false;
            RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Restore previous gait");
        } 
        else 
        {
            // Reset pending states if conditions not met
            pending_switch_to_stop_ = false;
            pending_restore_gait_ = false;
            switch_delay_counter_ = 0;
        }
    }
}
