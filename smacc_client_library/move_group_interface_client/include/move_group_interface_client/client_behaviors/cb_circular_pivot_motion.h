/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once 

#include "cb_move_end_effector_trajectory.h"
#include "tf/transform_datatypes.h"

namespace cl_move_group_interface
{
    class CbCircularPivotMotion : public CbMoveEndEffectorTrajectory
    {
    public:
        boost::optional<double> angularSpeed_rad_s_;
        boost::optional<double> linearSpeed_m_s_;

        // if not specified it would be used the current robot pose
        boost::optional<geometry_msgs::msg::Pose> relativeInitialPose_;

        CbCircularPivotMotion(std::string tipLink = "");

        CbCircularPivotMotion(const geometry_msgs::msg::PoseStamped &planePivotPose, double deltaRadians, std::string tipLink = "");

        CbCircularPivotMotion(const geometry_msgs::msg::PoseStamped &planePivotPose, const geometry_msgs::msg::Pose &relativeInitialPose, double deltaRadians, std::string tipLink = "");

        virtual void generateTrajectory() override;

        virtual void createMarkers() override;

    protected:
        geometry_msgs::msg::PoseStamped planePivotPose_;

        double deltaRadians_;

    private:
        void computeCurrentEndEffectorPoseRelativeToPivot();
    };

} // namespace cl_move_group_interface