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
    class CbCircularPouringMotion : public CbMoveEndEffectorTrajectory
    {
    public:
        boost::optional<double> angularSpeed_rad_s_;

        boost::optional<double> linearSpeed_m_s_;

        CbCircularPouringMotion(const geometry_msgs::msgs::Point &pivotPoint, double deltaHeight, std::string tipLink , std::string globalFrame);

        virtual void generateTrajectory() override;

        virtual void createMarkers() override;
        
        geometry_msgs::msgs::Vector3 directionVector_;

        // relative position of the "lid" of the bottle in the end effector reference frame.
        // that point is the one that must do the linear motion
        geometry_msgs::msgs::Pose pointerRelativePose_; 
    protected:
        geometry_msgs::msgs::Point relativePivotPoint_;

        // distance in meters from the initial pose to the bottom/top direction in z axe
        double deltaHeight_;

        std::vector<geometry_msgs::msgs::PoseStamped> pointerTrajectory_;

    private:
        void computeCurrentEndEffectorPoseRelativeToPivot();

        std::string globalFrame_;
    };

} // namespace cl_move_group_interface