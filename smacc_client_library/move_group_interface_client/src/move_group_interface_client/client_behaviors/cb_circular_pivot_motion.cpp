/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_circular_pivot_motion.h>

namespace cl_move_group_interface
{
    CbCircularPivotMotion::CbCircularPivotMotion(std::string tipLink)
        : CbMoveEndEffectorTrajectory(tipLink)
    {
    }

    CbCircularPivotMotion::CbCircularPivotMotion(const geometry_msgs::PoseStamped &planePivotPose, double deltaRadians, std::string tipLink)
        : planePivotPose_(planePivotPose), deltaRadians_(deltaRadians), CbMoveEndEffectorTrajectory(tipLink)
    {
    }

    CbCircularPivotMotion::CbCircularPivotMotion(const geometry_msgs::PoseStamped &planePivotPose, const geometry_msgs::Pose &relativeInitialPose, double deltaRadians, std::string tipLink)
        : planePivotPose_(planePivotPose), relativeInitialPose_(relativeInitialPose), deltaRadians_(deltaRadians), CbMoveEndEffectorTrajectory(tipLink)
    {
    }

    void CbCircularPivotMotion::generateTrajectory()
    {
        if (!relativeInitialPose_)
        {
            this->computeCurrentEndEffectorPoseRelativeToPivot();
        }

        // project offset into the xy-plane
        // get the radius
        double radius = sqrt(relativeInitialPose_->position.z * relativeInitialPose_->position.z + relativeInitialPose_->position.y * relativeInitialPose_->position.y);
        double initialAngle = atan2(relativeInitialPose_->position.z, relativeInitialPose_->position.y);

        double totallineardist = fabs(radius * deltaRadians_);
        double totalangulardist = fabs(deltaRadians_);

        // at least 1 sample per centimeter (average)
        // at least 1 sample per ~1.1 degrees (average)

        const double RADS_PER_SAMPLE = 0.02;
        const double METERS_PER_SAMPLE = 0.01;

        int totalSamplesCount = std::max(totallineardist / METERS_PER_SAMPLE, totalangulardist / RADS_PER_SAMPLE);

        double linearSecondsPerSample;
        double angularSecondsPerSamples;
        double secondsPerSample;

        if (linearSpeed_m_s_)
        {
            linearSecondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);
        }
        else
        {
            linearSecondsPerSample = std::numeric_limits<double>::max();
        }

        if (angularSpeed_rad_s_)
        {
            angularSecondsPerSamples = RADS_PER_SAMPLE / (*angularSpeed_rad_s_);
        }
        else
        {
            angularSecondsPerSamples = std::numeric_limits<double>::max();
        }

        if (!linearSpeed_m_s_ && !angularSpeed_rad_s_)
        {
            secondsPerSample = 0.5;
        }
        else
        {
            secondsPerSample = std::min(linearSecondsPerSample, angularSecondsPerSamples);
        }

        double currentAngle = initialAngle;

        double angleStep = deltaRadians_ / (double)totalSamplesCount;

        tf::Transform tfBasePose;
        tf::poseMsgToTF(planePivotPose_.pose, tfBasePose);

        for (int i = 0; i < totalSamplesCount; i++)
        {
            // relativePose i
            currentAngle += angleStep;
            double y = radius * cos(currentAngle);
            double z = radius * sin(currentAngle);

            geometry_msgs::Pose relativeCurrentPose;

            relativeCurrentPose.position.x = relativeInitialPose_->position.x;
            relativeCurrentPose.position.y = y;
            relativeCurrentPose.position.z = z;

            auto localquat = tf::createQuaternionFromRPY(currentAngle, 0, 0);
            //relativeCurrentPose.orientation = relativeInitialPose_.orientation;
            //tf::quaternionTFToMsg(localquat, relativeCurrentPose.orientation);
            relativeCurrentPose.orientation.w = 1;

            tf::Transform tfRelativeCurrentPose;
            tf::poseMsgToTF(relativeCurrentPose, tfRelativeCurrentPose);

            tf::Transform tfGlobalPose = tfRelativeCurrentPose * tfBasePose;

            tfGlobalPose.setRotation(tfGlobalPose.getRotation() * localquat);

            geometry_msgs::PoseStamped globalPose;
            tf::poseTFToMsg(tfGlobalPose, globalPose.pose);
            globalPose.header.frame_id = planePivotPose_.header.frame_id;
            globalPose.header.stamp = planePivotPose_.header.stamp + ros::Duration(i * secondsPerSample);

            this->endEffectorTrajectory_.push_back(globalPose);
        }
    }

    void CbCircularPivotMotion::computeCurrentEndEffectorPoseRelativeToPivot()
    {
        //auto currentRobotEndEffectorPose = this->movegroupClient_->moveGroupClientInterface.getCurrentPose();

        tf::TransformListener tfListener;
        // tf::StampedTransform globalBaseLink;
        tf::StampedTransform endEffectorInPivotFrame;

        try
        {
            if (!tipLink_ || *tipLink_ == "")
            {
                tipLink_ = this->movegroupClient_->moveGroupClientInterface.getEndEffectorLink();
            }

            tfListener.waitForTransform(planePivotPose_.header.frame_id, *tipLink_, ros::Time(0), ros::Duration(10));
            tfListener.lookupTransform(planePivotPose_.header.frame_id, *tipLink_, ros::Time(0), endEffectorInPivotFrame);

            // we define here the global frame as the pivot frame id
            // tfListener.waitForTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, ros::Time(0), ros::Duration(10));
            // tfListener.lookupTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, ros::Time(0), globalBaseLink);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        // tf::Transform endEffectorInBaseLinkFrame;
        // tf::poseMsgToTF(currentRobotEndEffectorPose.pose, endEffectorInBaseLinkFrame);

        // tf::Transform endEffectorInPivotFrame = globalBaseLink * endEffectorInBaseLinkFrame; // pose composition

        // now pivot and EndEffector share a common reference frame (let say map)
        // now get the current pose from the pivot reference frame with inverse composition
        tf::Transform pivotTransform;
        tf::poseMsgToTF(planePivotPose_.pose, pivotTransform);
        tf::Transform invertedNewReferenceFrame = pivotTransform.inverse();

        tf::Transform currentPoseRelativeToPivot = invertedNewReferenceFrame * endEffectorInPivotFrame;

        geometry_msgs::Pose finalEndEffectorRelativePose;
        tf::poseTFToMsg(currentPoseRelativeToPivot, finalEndEffectorRelativePose);
        relativeInitialPose_ = finalEndEffectorRelativePose;
    }

    void CbCircularPivotMotion::createMarkers()
    {
        CbMoveEndEffectorTrajectory::createMarkers();

        tf::Transform localdirection;
        localdirection.setIdentity();
        localdirection.setOrigin(tf::Vector3(0.12, 0, 0));
        auto frameid = planePivotPose_.header.frame_id;

        visualization_msgs::Marker marker;
        marker.header.frame_id = frameid;
        marker.header.stamp = nh_->now();();
        marker.ns = "trajectory";
        marker.id = beahiorMarkers_.markers.size();
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0;
        marker.color.b = 1.0;

        geometry_msgs::Point start, end;
        start.x = planePivotPose_.pose.position.x;
        start.y = planePivotPose_.pose.position.y;
        start.z = planePivotPose_.pose.position.z;

        tf::Transform basetransform;
        tf::poseMsgToTF(planePivotPose_.pose, basetransform);
        tf::Transform endarrow = localdirection * basetransform;

        end.x = endarrow.getOrigin().x();
        end.y = endarrow.getOrigin().y();
        end.z = endarrow.getOrigin().z();

        marker.pose.orientation.w = 1;
        marker.points.push_back(start);
        marker.points.push_back(end);

        beahiorMarkers_.markers.push_back(marker);
    }

} // namespace cl_move_group_interface