//
// Created by biao on 3/15/25.
//

#include "ocs2_quadruped_controller/control/CtrlComponent.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <angles/angles.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_quadruped_controller/estimator/FromOdomTopic.h>
#include <ocs2_quadruped_controller/estimator/GroundTruth.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>
#include <ocs2_quadruped_controller/estimator/ContactKalmanFilter.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot
{
    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces) : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        node_->declare_parameter("robot_pkg", robot_pkg_);
        node_->declare_parameter("feet", feet_names_);
        node_->declare_parameter("enable_perceptive", enable_perceptive_);
        node_->declare_parameter("enable_gait_adjust", enable_gait_adjust_);

        robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
        joint_names_ = node_->get_parameter("joints").as_string_array();
        feet_names_ = node_->get_parameter("feet").as_string_array();
        enable_perceptive_ = node_->get_parameter("enable_perceptive").as_bool();
        enable_gait_adjust_ = node_->get_parameter("enable_gait_adjust").as_bool();


        const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
        urdf_file_ = package_share_directory + "/urdf/robot.urdf";
        task_file_ = package_share_directory + "/config/ocs2/task.info";
        reference_file_ = package_share_directory + "/config/ocs2/reference.info";
        gait_file_ = package_share_directory + "/config/ocs2/gait.info";

        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);

        setupLeggedInterface();
        setupMpc();
        setupMrt();

        CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
        ee_kinematics_ = std::make_unique<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchio_mapping,
            legged_interface_->modelSettings().contactNames3DoF);

        rbd_conversions_ = std::make_unique<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                           legged_interface_->getCentroidalModelInfo());

        // Init visualizer
        visualizer_ = std::make_unique<LeggedRobotVisualizer>(
            legged_interface_->getPinocchioInterface(),
            legged_interface_->getCentroidalModelInfo(),
            *ee_kinematics_,
            node_);

        // Init observation
        observation_.state.setZero(static_cast<long>(legged_interface_->getCentroidalModelInfo().stateDim));
        observation_.input.setZero(
            static_cast<long>(legged_interface_->getCentroidalModelInfo().inputDim));
        observation_.mode = STANCE;
    }

    void CtrlComponent::setupStateEstimate(const std::string& estimator_type)
    {
        if (estimator_type == "ground_truth")
        {
            estimator_ = std::make_unique<GroundTruth>(legged_interface_->getCentroidalModelInfo(),
                                                       ctrl_interfaces_,
                                                       node_);
            RCLCPP_INFO(node_->get_logger(), "Using Ground Truth Estimator");
        }
        else if (estimator_type == "linear_kalman")
        {
            estimator_ = std::make_unique<KalmanFilterEstimate>(
                legged_interface_->getPinocchioInterface(),
                legged_interface_->getCentroidalModelInfo(),
                *ee_kinematics_, ctrl_interfaces_,
                node_);
            dynamic_cast<KalmanFilterEstimate&>(*estimator_).loadSettings(task_file_, verbose_);
            RCLCPP_INFO(node_->get_logger(), "Using Kalman Filter Estimator");
        }
        else if (estimator_type == "contact_kalman" || 
                 estimator_type == "gait_based_kalman")
        {
            estimator_ = std::make_unique<ContactKalmanFilterEstimate>(
                legged_interface_->getPinocchioInterface(),
                legged_interface_->getCentroidalModelInfo(),
                *ee_kinematics_, ctrl_interfaces_,
                node_, legged_interface_->getSwitchedModelReferenceManagerPtr(), observation_.time);
            dynamic_cast<ContactKalmanFilterEstimate&>(*estimator_).loadSettings(task_file_, verbose_);
            RCLCPP_INFO(node_->get_logger(), "Using Contact Kalman Filter Estimator");
        }
        else
        {
            estimator_ = std::make_unique<FromOdomTopic>(
                legged_interface_->getCentroidalModelInfo(), ctrl_interfaces_, node_);
            RCLCPP_INFO(node_->get_logger(), "Using Odom Topic Based Estimator");
        }
        observation_.time = 0;
    }

    void CtrlComponent::updateState(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Update State Estimation
        measured_rbd_state_ = estimator_->update(time, period);
        observation_.time += period.seconds();
        const scalar_t yaw_last = observation_.state(9);
        observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
        observation_.state(9) = yaw_last + angles::shortest_angular_distance(
            yaw_last, observation_.state(9));
        observation_.mode = estimator_->getMode();

        visualizer_->update(observation_);
        if (enable_perceptive_)
        {
            footPlacementVisualizationPtr_->update(observation_);
            sphereVisualizationPtr_->update(observation_);
        }

        // Compute target trajectory
        target_manager_->update(observation_);
        // Update the current state of the system
        mpc_mrt_interface_->setCurrentObservation(observation_);
    }

    void CtrlComponent::init()
    {
        if (mpc_running_ == false)
        {
            const TargetTrajectories target_trajectories({observation_.time},
                                                         {observation_.state},
                                                         {observation_.input});

            // Set the first observation and command and wait for optimization to finish
            mpc_mrt_interface_->setCurrentObservation(observation_);
            mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
            RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
            while (!mpc_mrt_interface_->initialPolicyReceived())
            {
                mpc_mrt_interface_->advanceMpc();
                rclcpp::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
            }
            RCLCPP_INFO(node_->get_logger(), "Initial policy has been received.");

            mpc_running_ = true;
        }
    }

    void CtrlComponent::setupLeggedInterface()
    {
        if (enable_perceptive_)
        {
            legged_interface_ = std::make_unique<PerceptiveLeggedInterface>(task_file_, urdf_file_, reference_file_);
        }
        else
        {
            legged_interface_ = std::make_unique<LeggedInterface>(task_file_, urdf_file_, reference_file_);
        }

        legged_interface_->setupJointNames(joint_names_, feet_names_);
        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);

        if (enable_perceptive_)
        {
            footPlacementVisualizationPtr_ = std::make_unique<FootPlacementVisualization>(
                *dynamic_cast<PerceptiveLeggedReferenceManager&>(*legged_interface_->getReferenceManagerPtr()).
                getConvexRegionSelectorPtr(),
                legged_interface_->getCentroidalModelInfo().numThreeDofContacts, node_);

            sphereVisualizationPtr_ = std::make_unique<SphereVisualization>(
                legged_interface_->getPinocchioInterface(), legged_interface_->getCentroidalModelInfo(),
                *dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPinocchioSphereInterfacePtr(), node_);
        }
    }

    /**
     * Set up the SQP MPC, Gait Manager and Reference Manager
     */
    void CtrlComponent::setupMpc()
    {
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(),
                                        legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());

        // Initialize the reference manager
        const auto gait_manager_ptr = std::make_shared<GaitManager>(
            ctrl_interfaces_,
            legged_interface_->getSwitchedModelReferenceManagerPtr()->
                               getGaitSchedule(),
            enable_gait_adjust_);
        gait_manager_ptr->init(gait_file_);
        mpc_->getSolverPtr()->addSynchronizedModule(gait_manager_ptr);
        mpc_->getSolverPtr()->setReferenceManager(legged_interface_->getReferenceManagerPtr());

        target_manager_ = std::make_unique<TargetManager>(ctrl_interfaces_,
                                                          node_,
                                                          legged_interface_->getReferenceManagerPtr(),
                                                          task_file_,
                                                          reference_file_,
                                                          gait_manager_ptr->in_stop_mode_);

        if (enable_perceptive_)
        {
            const auto planarTerrainReceiver =
                std::make_shared<PlanarTerrainReceiver>(
                    node_, dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPlanarTerrainPtr(),
                    dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getSignedDistanceFieldPtr(),
                    "/convex_plane_decomposition_ros/planar_terrain", "elevation");
            mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
        }
    }

    void CtrlComponent::setupMrt()
    {
        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
        mpc_timer_.reset();

        controller_running_ = true;
        mpc_thread_ = std::thread([&]
        {
            while (controller_running_)
            {
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (mpc_running_)
                            {
                                mpc_timer_.startTimer();
                                mpc_mrt_interface_->advanceMpc();
                                mpc_timer_.endTimer();
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                }
                catch (const std::exception& e)
                {
                    controller_running_ = false;
                    RCLCPP_WARN(node_->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
        RCLCPP_INFO(node_->get_logger(), "MRT initialized. MPC thread started.");
    }
    
    void CtrlComponent::enableAMPDataPublisher(bool enable, double publish_rate)
    {
        if (enable && !enable_amp_publisher_)
        {
            enable_amp_publisher_ = true;
            
            // Create publisher
            amp_data_pub_ = node_->create_publisher<control_input_msgs::msg::AMPMotionData>(
                "/amp_motion_data", 10);
            
            // Create timer for publishing at specified rate
            auto period = std::chrono::duration<double>(1.0 / publish_rate);
            amp_publish_timer_ = node_->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                [this]() { publishAMPData(); });
            
            RCLCPP_INFO(node_->get_logger(), "AMP Data Publisher enabled at %.1f Hz", publish_rate);
        }
        else if (!enable && enable_amp_publisher_)
        {
            enable_amp_publisher_ = false;
            amp_publish_timer_.reset();
            amp_data_pub_.reset();
            RCLCPP_INFO(node_->get_logger(), "AMP Data Publisher disabled");
        }
    }
    
    void CtrlComponent::publishAMPData()
    {
        if (!enable_amp_publisher_ || !mpc_running_)
        {
            return;
        }

        // Safety check: ensure ee_kinematics_ is initialized
        if (!ee_kinematics_)
        {
            return;
        }
        
        // Ensure Pinocchio interface is set (required before using getPosition/getVelocity)
        ee_kinematics_->setPinocchioInterface(legged_interface_->getPinocchioInterface());

        try
        {
            auto msg = control_input_msgs::msg::AMPMotionData();
        
            // Get current time
            msg.timestamp = node_->get_clock()->now();
        
            // Extract root position (base position from state)
            // Centroidal state: [normalized_momentum(6), base_ori_euler(3), base_pos(3), joint_pos(12)]
            // But we need RBD state for velocities: [base_ori(3), base_pos(3), joint_pos(12), base_ang_vel(3), base_lin_vel(3), joint_vel(12)]
            const auto& info = legged_interface_->getCentroidalModelInfo();
            const int generalizedCoordinatesNum = info.generalizedCoordinatesNum;  // 18 = 6 (base) + 12 (joints)
            
            // Extract from measured_rbd_state_ (RBD state with velocities)
            msg.root_pos[0] = measured_rbd_state_(3);  // x
            msg.root_pos[1] = measured_rbd_state_(4);  // y
            msg.root_pos[2] = measured_rbd_state_(5);  // z
        
            // Extract root orientation (convert from Euler ZYX to quaternion)
            const double roll = measured_rbd_state_(0);
            const double pitch = measured_rbd_state_(1);
            const double yaw = measured_rbd_state_(2);
        
            // Convert Euler ZYX to quaternion
            const double cy = std::cos(yaw * 0.5);
            const double sy = std::sin(yaw * 0.5);
            const double cp = std::cos(pitch * 0.5);
            const double sp = std::sin(pitch * 0.5);
            const double cr = std::cos(roll * 0.5);
            const double sr = std::sin(roll * 0.5);
        
            msg.root_quat[0] = sr * cp * cy - cr * sp * sy;  // x
            msg.root_quat[1] = cr * sp * cy + sr * cp * sy;  // y
            msg.root_quat[2] = cr * cp * sy - sr * sp * cy;  // z
            msg.root_quat[3] = cr * cp * cy + sr * sp * sy;  // w
        
            // Extract joint positions (12 DOF)
            for (int i = 0; i < 12; ++i)
            {
                msg.joint_pos[i] = measured_rbd_state_(6 + i);  // Joint positions start at index 6 in RBD state
            }
        
            // Compute foot positions in base frame using forward kinematics
            const std::vector<Eigen::Vector3d> ee_positions = ee_kinematics_->getPosition(observation_.state);
            for (size_t leg = 0; leg < 4; ++leg)
            {
                msg.foot_pos_local[leg * 3 + 0] = ee_positions[leg](0);
                msg.foot_pos_local[leg * 3 + 1] = ee_positions[leg](1);
                msg.foot_pos_local[leg * 3 + 2] = ee_positions[leg](2);
            }
        
            // Extract linear velocity (base frame) - from RBD state
            msg.linear_vel[0] = measured_rbd_state_(generalizedCoordinatesNum + 3);   // vx
            msg.linear_vel[1] = measured_rbd_state_(generalizedCoordinatesNum + 4);   // vy
            msg.linear_vel[2] = measured_rbd_state_(generalizedCoordinatesNum + 5);   // vz
        
            // Extract angular velocity (base frame) - from RBD state
            msg.angular_vel[0] = measured_rbd_state_(generalizedCoordinatesNum + 0);   // wx
            msg.angular_vel[1] = measured_rbd_state_(generalizedCoordinatesNum + 1);   // wy
            msg.angular_vel[2] = measured_rbd_state_(generalizedCoordinatesNum + 2);   // wz
        
            // Extract joint velocities (12 DOF) - from RBD state
            for (int i = 0; i < 12; ++i)
            {
                msg.joint_vel[i] = measured_rbd_state_(generalizedCoordinatesNum + 6 + i);  // Joint velocities start after base velocities
            }
        
            // Compute foot velocities in base frame using forward kinematics
            const std::vector<Eigen::Vector3d> ee_velocities = ee_kinematics_->getVelocity(observation_.state, observation_.input);
            for (size_t leg = 0; leg < 4; ++leg)
            {
                msg.foot_vel_local[leg * 3 + 0] = ee_velocities[leg](0);
                msg.foot_vel_local[leg * 3 + 1] = ee_velocities[leg](1);
                msg.foot_vel_local[leg * 3 + 2] = ee_velocities[leg](2);
            }
        
            // Publish the message
            amp_data_pub_->publish(msg);
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "AMP data publishing failed: %s", e.what());
            return;
        }
    }
}

