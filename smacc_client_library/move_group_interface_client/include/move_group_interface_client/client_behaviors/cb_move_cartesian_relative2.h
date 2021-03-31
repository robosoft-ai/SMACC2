/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <move_group_interface_client/cl_movegroup.h>
#include <smacc/smacc_asynchronous_client_behavior.h>
#include <tf/transform_datatypes.h>

namespace cl_move_group_interface
{
    class CbMoveCartesianRelative2 : public CbMoveEndEffectorTrajectory
    {
    public:
        geometry_msgs::msg::Vector3 offset_;

        boost::optional<double> linearSpeed_m_s_;

        CbMoveCartesianRelative2(std::string referenceFrame, std::string tipLink)
            : CbMoveEndEffectorTrajectory(tipLink)
        {
            globalFrame_ = referenceFrame;
        }

        CbMoveCartesianRelative2(std::string referenceFrame, std::string tipLink, geometry_msgs::msg::Vector3 offset)
            : CbMoveEndEffectorTrajectory(tipLink)
        {
            globalFrame_ = referenceFrame;
            offset_ = offset;
        }

        virtual void generateTrajectory() override
        {
            // at least 1 sample per centimeter (average)
            const double METERS_PER_SAMPLE = 0.001;

            float dist_meters = 0;
            tf::Vector3 voffset;
            tf::vector3MsgToTF(offset_, voffset);

            float totallineardist = voffset.length();

            int totalSamplesCount = totallineardist / METERS_PER_SAMPLE;
            int steps = totallineardist / METERS_PER_SAMPLE;

            float interpolation_factor_step = 1.0 / totalSamplesCount;

            double secondsPerSample;

            if (!linearSpeed_m_s_)
            {
                linearSpeed_m_s_ = 0.1;
            }

            secondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);

            tf::StampedTransform currentEndEffectorTransform;

            this->getCurrentEndEffectorPose(globalFrame_, currentEndEffectorTransform);

            tf::Transform finalEndEffectorTransform = currentEndEffectorTransform;
            finalEndEffectorTransform.setOrigin(finalEndEffectorTransform.getOrigin() + voffset);

            float linc = totallineardist / steps; // METERS_PER_SAMPLE with sign
            float interpolation_factor = 0;

            ros::Time startTime = nh_->now();();

            for (float i = 0; i < steps; i++)
            {
                interpolation_factor += interpolation_factor_step;
                dist_meters += linc;

                tf::Vector3 vi = currentEndEffectorTransform.getOrigin().lerp(finalEndEffectorTransform.getOrigin(), interpolation_factor);

                tf::Transform pose;
                pose.setOrigin(vi);
                pose.setRotation(currentEndEffectorTransform.getRotation());

                geometry_msgs::msg::PoseStamped pointerPose;
                tf::poseTFToMsg(pose, pointerPose.pose);
                pointerPose.header.frame_id = globalFrame_;
                pointerPose.header.stamp = startTime + ros::Duration(i * secondsPerSample);

                this->endEffectorTrajectory_.push_back(pointerPose);
            }

            RCLCPP_INFO_STREAM(nh_->get_logger(), "[CbMoveEndEffectorRelative2] Target End efector Pose: " << this->endEffectorTrajectory_.back());

        }

    private:
        std::string globalFrame_;
    };
} // namespace cl_move_group_interface