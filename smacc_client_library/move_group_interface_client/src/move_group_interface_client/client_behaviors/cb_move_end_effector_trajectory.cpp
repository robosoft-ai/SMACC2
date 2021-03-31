/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_end_effector_trajectory.h>
#include <moveit_srvs/srv/get_position_ik.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf/transform_datatypes.h>
#include <move_group_interface_client/components/cp_trajectory_history.h>
#include <tf2/transform_listener.h>

namespace cl_move_group_interface
{
    CbMoveEndEffectorTrajectory::CbMoveEndEffectorTrajectory(std::string tipLink)
        : tipLink_(tipLink), markersInitialized_(false)
    {
        initializeROS();
    }

    CbMoveEndEffectorTrajectory::CbMoveEndEffectorTrajectory(const std::vector<geometry_msgs::PoseStamped> &endEffectorTrajectory, std::string tipLink)
        : endEffectorTrajectory_(endEffectorTrajectory), tipLink_(tipLink), markersInitialized_(false)

    {
        initializeROS();
    }

    void CbMoveEndEffectorTrajectory::initializeROS()
    {
        ros::NodeHandle nh;
        markersPub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1);
        iksrv_ = nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
    }

    ComputeJointTrajectoryErrorCode CbMoveEndEffectorTrajectory::computeJointSpaceTrajectory(moveit_msgs::RobotTrajectory &computedJointTrajectory)
    {
        // get current robot state
        auto currentState = movegroupClient_->moveGroupClientInterface.getCurrentState();

        // get the IK client
        auto groupname = movegroupClient_->moveGroupClientInterface.getName();
        auto currentjointnames = currentState->getJointModelGroup(groupname)->getActiveJointModelNames();

        if (!tipLink_ || *tipLink_ == "")
        {
            tipLink_ = movegroupClient_->moveGroupClientInterface.getEndEffectorLink();
        }

        std::vector<double> jointPositions;
        currentState->copyJointGroupPositions(groupname, jointPositions);

        std::vector<std::vector<double>> trajectory;
        std::vector<ros::Duration> trajectoryTimeStamps;

        trajectory.push_back(jointPositions);
        trajectoryTimeStamps.push_back(ros::Duration(0));

        auto &first = endEffectorTrajectory_.front();
        ros::Time referenceTime = first.header.stamp;

        std::vector<int> discontinuityIndexes;

        int ikAttempts = 4;
        for (int k = 0; k < this->endEffectorTrajectory_.size(); k++)
        {
            auto &pose = this->endEffectorTrajectory_[k];
            moveit_msgs::GetPositionIKRequest req;
            //req.ik_request.attempts = 20;
            
            req.ik_request.ik_link_name = *tipLink_;
            req.ik_request.robot_state.joint_state.name = currentjointnames;
            req.ik_request.robot_state.joint_state.position = jointPositions;

            req.ik_request.group_name = groupname;
            req.ik_request.avoid_collisions = true;

            moveit_msgs::GetPositionIKResponse res;

            //pose.header.stamp = nh_->now();();
            req.ik_request.pose_stamped = pose;

            RCLCPP_DEBUG_STREAM(nh_->get_logger(), "IK request: " << req);
            if (iksrv_.call(req, res))
            {
                auto &prevtrajpoint = trajectory.back();
                //jointPositions.clear();

                std::stringstream ss;
                for (int j = 0; j < res.solution.joint_state.position.size(); j++)
                {
                    auto &jointname = res.solution.joint_state.name[j];
                    auto it = std::find(currentjointnames.begin(), currentjointnames.end(), jointname);
                    if (it != currentjointnames.end())
                    {
                        int index = std::distance(currentjointnames.begin(), it);
                        jointPositions[index] = res.solution.joint_state.position[j];
                        ss << jointname << "(" << index << "): " << jointPositions[index] << std::endl;
                    }
                }

                // continuity check
                int jointindex = 0;
                int discontinuityJointIndex = -1;
                double discontinuityDeltaJointIndex = -1;
                double deltajoint;

                bool check = k > 0 || !allowInitialTrajectoryStateJointDiscontinuity_ || allowInitialTrajectoryStateJointDiscontinuity_ && !(*allowInitialTrajectoryStateJointDiscontinuity_);
                if (check)
                {
                    for (jointindex = 0; jointindex < jointPositions.size(); jointindex++)
                    {
                        deltajoint = jointPositions[jointindex] - prevtrajpoint[jointindex];

                        if (fabs(deltajoint) > 0.3 /*2.5 deg*/)
                        {
                            discontinuityDeltaJointIndex = deltajoint;
                            discontinuityJointIndex = jointindex;
                        }
                    }
                }

                if (ikAttempts > 0 && discontinuityJointIndex != -1)
                {
                    k--;
                    ikAttempts--;
                    continue;
                }
                else
                {
                    bool discontinuity = false;
                    if (ikAttempts == 0)
                    {
                        discontinuityIndexes.push_back(k);
                        discontinuity = true;
                    }

                    ikAttempts = 4;

                    if (discontinuity && discontinuityJointIndex != -1)
                    {
                        // show a message and stop the trajectory generation && jointindex!= 7 || fabs(deltajoint) > 0.1 /*2.5 deg*/  && jointindex== 7
                        std::stringstream ss;
                        ss << "Traj[" << k << "/" << endEffectorTrajectory_.size() << "] " << currentjointnames[discontinuityJointIndex] << " IK discontinuity : " << discontinuityDeltaJointIndex << std::endl
                           << "prev joint value: " << prevtrajpoint[discontinuityJointIndex] << std::endl
                           << "current joint value: " << jointPositions[discontinuityJointIndex] << std::endl;

                        ss << std::endl;
                        for (int ji = 0; ji < jointPositions.size(); ji++)
                        {
                            ss << currentjointnames[ji] << ": " << jointPositions[ji] << std::endl;
                        }

                        for (int kindex = 0; kindex < trajectory.size(); kindex++)
                        {
                            ss << "[" << kindex << "]: " << trajectory[kindex][discontinuityJointIndex] << std::endl;
                        }

                        if (k == 0)
                        {
                            ss << "This is the first posture of the trajectory. Maybe the robot initial posture is not coincident to the initial posture of the generated joint trajectory." << std::endl;
                        }

                        ROS_ERROR_STREAM(ss.str());

                        trajectory.push_back(jointPositions);
                        ros::Duration durationFromStart = pose.header.stamp - referenceTime;
                        trajectoryTimeStamps.push_back(durationFromStart);

                        continue;
                    }
                    else
                    {
                        trajectory.push_back(jointPositions);
                        ros::Duration durationFromStart = pose.header.stamp - referenceTime;
                        trajectoryTimeStamps.push_back(durationFromStart);

                        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "IK solution: " << res.solution.joint_state);
                        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "trajpoint: " << std::endl
                                                      << ss.str());
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(nh_->get_logger(), "[CbMoveEndEffectorTrajectory] wrong IK call");
            }

            RCLCPP_WARN_STREAM(nh_->get_logger(), "-----");
        }

        // interpolate speeds?

        // interpolate accelerations?

        // get current robot state
        // fill plan message
        // computedMotionPlan.start_state_.joint_state.name = currentjointnames;
        // computedMotionPlan.start_state_.joint_state.position = trajectory.front();
        // computedMotionPlan.trajectory_.joint_trajectory.joint_names = currentjointnames;

        computedJointTrajectory.joint_trajectory.joint_names = currentjointnames;
        int i = 0;
        for (auto &p : trajectory)
        {
            if (i == 0) // not copy the current state in the trajectory (used to solve discontinuity in other behaviors)
            {
                i++;
                continue;
            }

            trajectory_msgs::JointTrajectoryPoint jp;
            jp.positions = p;
            jp.time_from_start = trajectoryTimeStamps[i]; //ros::Duration(t);
            computedJointTrajectory.joint_trajectory.points.push_back(jp);
            i++;
        }

        if (discontinuityIndexes.size())
        {
            if (discontinuityIndexes[0] == 0)
                return ComputeJointTrajectoryErrorCode::INCORRECT_INITIAL_STATE;
            else
                return ComputeJointTrajectoryErrorCode::JOINT_TRAJECTORY_DISCONTINUITY;
        }

        return ComputeJointTrajectoryErrorCode::SUCCESS;
    }

    void CbMoveEndEffectorTrajectory::executeJointSpaceTrajectory(const moveit_msgs::RobotTrajectory &computedJointTrajectory)
    {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << this->getName() << "] Executing joint trajectory");
        // call execute
        auto executionResult = this->movegroupClient_->moveGroupClientInterface.execute(computedJointTrajectory);

        if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << this->getName() << "] motion execution succedded");
            movegroupClient_->postEventMotionExecutionSucceded();
            this->postSuccessEvent();
        }
        else
        {
            this->postMotionExecutionFailureEvents();
            this->postFailureEvent();
        }
    }

    void CbMoveEndEffectorTrajectory::onEntry()
    {
        this->requiresClient(movegroupClient_);

        this->generateTrajectory();

        if (this->endEffectorTrajectory_.size() == 0)
        {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << smacc::demangleSymbol(typeid(*this).name()) << "] No points in the trajectory. Skipping behavior.");
            return;
        }

        this->createMarkers();
        markersInitialized_ = true;

        moveit_msgs::RobotTrajectory computedTrajectory;

        auto errorcode = computeJointSpaceTrajectory(computedTrajectory);

        bool trajectoryGenerationSuccess = errorcode == ComputeJointTrajectoryErrorCode::SUCCESS;

        CpTrajectoryHistory *trajectoryHistory;
        this->requiresComponent(trajectoryHistory);

        if (!trajectoryGenerationSuccess)
        {
            RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << this->getName() << "] Incorrect trajectory. Posting failure event.");
            if (trajectoryHistory != nullptr)
            {
                moveit_msgs::MoveItErrorCodes error;
                error.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
                trajectoryHistory->pushTrajectory(this->getName(), computedTrajectory, error);
            }

            movegroupClient_->postEventMotionExecutionFailed();
            this->postFailureEvent();

            if (errorcode == ComputeJointTrajectoryErrorCode::JOINT_TRAJECTORY_DISCONTINUITY)
            {
                this->postJointDiscontinuityEvent(computedTrajectory);
            }
            else if (errorcode == ComputeJointTrajectoryErrorCode::INCORRECT_INITIAL_STATE)
            {
                this->postIncorrectInitialStateEvent(computedTrajectory);
            }
            return;
        }
        else
        {
            if (trajectoryHistory != nullptr)
            {
                moveit_msgs::MoveItErrorCodes error;
                error.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
                trajectoryHistory->pushTrajectory(this->getName(), computedTrajectory, error);
            }

            this->executeJointSpaceTrajectory(computedTrajectory);
        }

        // handle finishing events
    } // namespace cl_move_group_interface

    void CbMoveEndEffectorTrajectory::update()
    {
        if (markersInitialized_)
        {
            std::lock_guard<std::mutex> guard(m_mutex_);
            markersPub_.publish(beahiorMarkers_);
        }
    }

    void CbMoveEndEffectorTrajectory::onExit()
    {
        markersInitialized_ = false;

        if (autocleanmarkers)
        {
            std::lock_guard<std::mutex> guard(m_mutex_);
            for (auto &marker : this->beahiorMarkers_.markers)
            {
                marker.header.stamp = nh_->now();();
                marker.action = visualization_msgs::Marker::DELETE;
            }

            markersPub_.publish(beahiorMarkers_);
        }
    }

    void CbMoveEndEffectorTrajectory::createMarkers()
    {
        tf::Transform localdirection;
        localdirection.setIdentity();
        localdirection.setOrigin(tf::Vector3(0.05, 0, 0));
        auto frameid = this->endEffectorTrajectory_.front().header.frame_id;

        for (auto &pose : this->endEffectorTrajectory_)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frameid;
            marker.header.stamp = nh_->now();();
            marker.ns = "trajectory";
            marker.id = this->beahiorMarkers_.markers.size();
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.005;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 0.8;
            marker.color.r = 1.0;
            marker.color.g = 0;
            marker.color.b = 0;

            geometry_msgs::Point start, end;
            start.x = 0;
            start.y = 0;
            start.z = 0;

            tf::Transform basetransform;
            tf::poseMsgToTF(pose.pose, basetransform);
            tf::Transform endarrow = localdirection * basetransform;

            end.x = localdirection.getOrigin().x();
            end.y = localdirection.getOrigin().y();
            end.z = localdirection.getOrigin().z();

            marker.pose.position = pose.pose.position;
            marker.pose.orientation = pose.pose.orientation;
            marker.points.push_back(start);
            marker.points.push_back(end);

            beahiorMarkers_.markers.push_back(marker);
        }
    }

    void CbMoveEndEffectorTrajectory::generateTrajectory()
    {
        // bypass current trajectory, overriden in derived classes
        // this->endEffectorTrajectory_ = ...
    }

    void CbMoveEndEffectorTrajectory::getCurrentEndEffectorPose(std::string globalFrame, tf::StampedTransform &currentEndEffectorTransform)
    {
        tf::TransformListener tfListener;

        try
        {
            if (!tipLink_ || *tipLink_ == "")
            {
                tipLink_ = this->movegroupClient_->moveGroupClientInterface.getEndEffectorLink();
            }

            tfListener.waitForTransform(globalFrame, *tipLink_, ros::Time(0), ros::Duration(10));
            tfListener.lookupTransform(globalFrame, *tipLink_, ros::Time(0), currentEndEffectorTransform);

            // we define here the global frame as the pivot frame id
            // tfListener.waitForTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, ros::Time(0), ros::Duration(10));
            // tfListener.lookupTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, ros::Time(0), globalBaseLink);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
} // namespace cl_move_group_interface