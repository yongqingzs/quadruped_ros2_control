//
// Created by qiayuan on 2022/7/24.
// Adapted by Copilot on 2025/9/3.
//

#pragma once

#include "StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>

#include <tf2_ros/transform_broadcaster.h>

namespace ocs2::legged_robot
{
    class GaitBasedKalmanFilter final : public StateEstimateBase
    {
    public:
        GaitBasedKalmanFilter(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                              const PinocchioEndEffectorKinematics& ee_kinematics,
                              CtrlInterfaces& ctrl_interfaces,
                              const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                              const std::shared_ptr<SwitchedModelReferenceManager>& referenceManagerPtr);

        vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        void loadSettings(const std::string& task_file, bool verbose);

    protected:
        nav_msgs::msg::Odometry getOdomMsg();

        PinocchioInterface pinocchio_interface_;
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
        std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

        vector_t feet_heights_;

        // Config
        scalar_t foot_radius_ = 0.02;
        scalar_t imu_process_noise_position_ = 0.02;
        scalar_t imu_process_noise_velocity_ = 0.02;
        scalar_t footProcessNoisePosition_ = 0.002;
        scalar_t footSensorNoisePosition_ = 0.005;
        scalar_t footSensorNoiseVelocity_ = 0.1;
        scalar_t footHeightSensorNoise_ = 0.01;

    private:
        size_t numContacts_, dimContacts_, numState_, numObserve_;

        matrix_t a_, b_, c_, q_, p_, r_;
        vector_t xHat_, ps_, vs_;
    };
} // namespace ocs2::legged_robot
