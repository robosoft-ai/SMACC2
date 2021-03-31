#include <move_group_interface_client/client_behaviors/cb_pouring_motion.h>

/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_circular_pivot_motion.h>

namespace cl_move_group_interface
{
    CbCircularPouringMotion::CbCircularPouringMotion(const geometry_msgs::Point &relativePivotPoint, double deltaHeight, std::string tipLink, std::string globalFrame)
        : relativePivotPoint_(relativePivotPoint)
        , deltaHeight_(deltaHeight)
        , CbMoveEndEffectorTrajectory(tipLink)
        , globalFrame_(globalFrame)
    {
    }

    void CbCircularPouringMotion::generateTrajectory()
    {
        // at least 1 sample per centimeter (average)        
        const double METERS_PER_SAMPLE = 0.001;

        float dist_meters =0;
        float totallineardist = fabs(this->deltaHeight_);

        int totalSamplesCount = totallineardist / METERS_PER_SAMPLE;
        int steps = totallineardist/ METERS_PER_SAMPLE;

        float interpolation_factor_step = 1.0/totalSamplesCount;

        double secondsPerSample;

        if (linearSpeed_m_s_)
        {
            secondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);
        }
        else
        {
            secondsPerSample = std::numeric_limits<double>::max();
        }

        tf::StampedTransform currentEndEffectorTransform;

        this->getCurrentEndEffectorPose(globalFrame_, currentEndEffectorTransform);

        tf::Transform lidEndEffectorTransform;
        tf::poseMsgToTF(this->pointerRelativePose_, lidEndEffectorTransform);

        tf::Vector3 v0, v1;
        v0 = (currentEndEffectorTransform * lidEndEffectorTransform).getOrigin();

        tf::Point pivotPoint;
        tf::pointMsgToTF(this->relativePivotPoint_, pivotPoint);

        tf::Vector3 pivot = (currentEndEffectorTransform * pivotPoint);
        
        v0 = v0 - pivot;
        v1 = v0;
        v1.setZ(v1.z() + this->deltaHeight_);

        tf::Vector3 vp1(v1);
        tf::Vector3 vp0(v0);
        tf::Quaternion rotation = tf::shortestArcQuatNormalize2(vp0, vp1);

        tf::Quaternion initialEndEffectorOrientation = currentEndEffectorTransform.getRotation();
        auto finalEndEffectorOrientation = initialEndEffectorOrientation* rotation;

        tf::Quaternion initialPointerOrientation = initialEndEffectorOrientation * lidEndEffectorTransform.getRotation();
        tf::Quaternion finalPointerOrientation = finalEndEffectorOrientation * lidEndEffectorTransform.getRotation();

        auto shortestAngle = tf::angleShortestPath(initialEndEffectorOrientation, finalEndEffectorOrientation);

        v0+=pivot;
        v1+=pivot;

        float linc = deltaHeight_/steps;// METERS_PER_SAMPLE with sign
        float interpolation_factor = 0;
        tf::Vector3 vi = v0;

        tf::Transform invertedLidTransform = lidEndEffectorTransform.inverse();
        ros::Time startTime = nh_->now();();

        for (float i =0; i < steps; i++)
        {
            auto currentEndEffectorOrientation = tf::slerp(initialEndEffectorOrientation, finalEndEffectorOrientation, interpolation_factor);
            auto currentPointerOrientation = tf::slerp(initialPointerOrientation, finalPointerOrientation, interpolation_factor);

            interpolation_factor += interpolation_factor_step;
            dist_meters += linc;

            vi = v0;
            vi.setZ(vi.getZ() + dist_meters);

            tf::Transform pose;
            pose.setOrigin(vi);
            pose.setRotation(currentPointerOrientation);

            geometry_msgs::PoseStamped pointerPose;
            tf::poseTFToMsg(pose, pointerPose.pose);
            pointerPose.header.frame_id = globalFrame_;
            pointerPose.header.stamp = startTime + ros::Duration(i * secondsPerSample);
            this->pointerTrajectory_.push_back(pointerPose);

            tf::Transform poseEndEffector = pose * invertedLidTransform;

            geometry_msgs::PoseStamped globalEndEffectorPose;
            tf::poseTFToMsg(poseEndEffector, globalEndEffectorPose.pose);
            globalEndEffectorPose.header.frame_id = globalFrame_;
            globalEndEffectorPose.header.stamp = startTime + ros::Duration(i * secondsPerSample);

            this->endEffectorTrajectory_.push_back(globalEndEffectorPose);
        }
    }

    void CbCircularPouringMotion::createMarkers()
    {
        CbMoveEndEffectorTrajectory::createMarkers();

        tf::StampedTransform currentEndEffectorTransform;
        this->getCurrentEndEffectorPose(globalFrame_, currentEndEffectorTransform);
        tf::Point pivotPoint;
        tf::pointMsgToTF(this->relativePivotPoint_, pivotPoint);
        tf::Vector3 pivot = (currentEndEffectorTransform * pivotPoint);


        visualization_msgs::Marker marker;

        marker.ns = "trajectory";
        marker.id = beahiorMarkers_.markers.size();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0;
        marker.color.b = 1.0;

        tf::pointTFToMsg(pivot, marker.pose.position);
        marker.header.frame_id = globalFrame_;
        marker.header.stamp = nh_->now();();

        beahiorMarkers_.markers.push_back(marker);
 
        tf::Transform localdirection;
        localdirection.setIdentity();
        localdirection.setOrigin(tf::Vector3(0.05, 0, 0));
        auto frameid = this->pointerTrajectory_.front().header.frame_id;

        for (auto &pose : this->pointerTrajectory_)
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
            marker.color.r = 0.0;
            marker.color.g = 1;
            marker.color.b = 0.0;

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

} // namespace cl_move_group_interface