//
// Created by chenchen on 2025/10/29.
//

#pragma once

#include "StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>

#include <tf2_ros/transform_broadcaster.h>
#include <control_input_msgs/msg/contact_info.hpp>

namespace ocs2::legged_robot {
    class ContactKalmanFilterEstimate final : public StateEstimateBase {
    public:
        ContactKalmanFilterEstimate(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                        const PinocchioEndEffectorKinematics& ee_kinematics,
                        CtrlInterfaces& ctrl_interfaces,
                        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                        const std::shared_ptr<SwitchedModelReferenceManager>& referenceManagerPtr,
                        const scalar_t& obs_time);        
                        
        vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        void loadSettings(const std::string &task_file, bool verbose);

        void updateJointStates() override;

        vector_t estContactForce(const rclcpp::Duration &period);

        void updateEstContact(const rclcpp::Duration &period);
        
        bool verbose_ = false;

    protected:
        nav_msgs::msg::Odometry getOdomMsg();

        PinocchioInterface pinocchio_interface_;
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
        std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;
        const scalar_t& obs_time_;

        vector_t feet_heights_;
        
        scalar_t cutoffFrequency_ = 150;
        vector_t jointTor_;
        vector_t pSCgZinvlast_;
        int est_contact_threshold_ = 40;
        int iter_ = 0;

        // Contact info publisher
        rclcpp::Publisher<control_input_msgs::msg::ContactInfo>::SharedPtr contact_info_publisher_;

        // Config
        scalar_t foot_radius_ = 0.02;
        scalar_t imu_process_noise_position_ = 0.02;
        scalar_t imu_process_noise_velocity_ = 0.02;
        scalar_t footProcessNoisePosition_ = 0.002;
        scalar_t footSensorNoisePosition_ = 0.005;
        scalar_t footSensorNoiseVelocity_ = 0.1;
        scalar_t footHeightSensorNoise_ = 0.01;

        // External
        std::string estimator_type_ = "gait_based_kalman";

    private:
        size_t numContacts_, dimContacts_, numState_, numObserve_;

        matrix_t a_, b_, c_, q_, p_, r_;
        vector_t xHat_, ps_, vs_;
    };
}
