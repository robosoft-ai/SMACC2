/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
// #include <ros/ros.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>

#include <angles/angles.h>
   
namespace cl_move_base_z
{
    geometry_msgs::msg::PoseStamped makePureSpinningSubPlan(const geometry_msgs::msg::PoseStamped& start, double dstRads, std::vector<geometry_msgs::msg::PoseStamped>& plan, double puresSpinningRadStep=1000)
    {
        double startYaw = tf2::getYaw(start.pose.orientation);
        //ROS_INFO("pure spining start yaw: %lf", startYaw);
        //ROS_INFO("pure spining goal yaw: %lf", dstRads);
        //ROS_WARN_STREAM("pure spinning start pose: " << start);

        double goalAngleOffset = angles::shortest_angular_distance(startYaw, dstRads);
        //ROS_INFO("shortest angle: %lf", goalAngleOffset);

        double radstep = 0.005;

        if( goalAngleOffset>=0) 
        {
            // angle positive turn counterclockwise
            //ROS_INFO("pure spining counterclockwise");
            for(double dangle = 0; dangle <= goalAngleOffset;dangle+=radstep )
            {
                geometry_msgs::msg::PoseStamped p= start;
                double yaw = startYaw+ dangle;
                //ROS_INFO("pure spining counterclockwise, current path yaw: %lf, dangle: %lf, angleoffset %lf, radstep %lf pathsize(%ld)", yaw, dangle, goalAngleOffset, radstep, plan.size());
                tf2::Quaternion q;
                q.setEuler(yaw,0,0);
                p.pose.orientation = tf2::toMsg(q);
                plan.push_back(p);
            }     
        }
        else
        {
            // angle positive turn clockwise
            //ROS_INFO("pure spining clockwise");
            for(double dangle = 0; dangle >= goalAngleOffset;dangle-=radstep )
            {
                //ROS_INFO("dangle: %lf", dangle);
                geometry_msgs::msg::PoseStamped p= start;
                double yaw = startYaw+ dangle;
                //ROS_INFO("pure spining clockwise, yaw: %lf, dangle: %lf, angleoffset %lf radstep %lf", yaw, dangle, goalAngleOffset,radstep);
                tf2::Quaternion q;
                q.setEuler(yaw,0,0);
                p.pose.orientation = tf2::toMsg(q);
                plan.push_back(p);
            }     
        }

        //ROS_INFO("pure spining end yaw: %lf", dstRads);        
        geometry_msgs::msg::PoseStamped end= start;
        tf2::Quaternion q;
        q.setEuler(dstRads,0,0);
        end.pose.orientation = tf2::toMsg(q);
        plan.push_back(end);

        return end;
    }

    geometry_msgs::msg::PoseStamped makePureStraightSubPlan(const geometry_msgs::msg::PoseStamped& startOrientedPose, 
                                                            const geometry_msgs::msg::Point& goal, 
                                                            double lenght, 
                                                            std::vector<geometry_msgs::msg::PoseStamped>& plan)
    {
        double dx = 0.01; // 1 cm
        double steps = lenght /dx;
        double dt = 1.0/steps;

        // geometry_msgs::msg::PoseStamped end;
        // end.pose.orientation = startOrientedPose.pose.orientation;
        //end.pose.position = goal;
        plan.push_back(startOrientedPose);

        //ROS_INFO_STREAM("Pure straight, start: " << startOrientedPose << std::endl << "end: " << goal);
        for(double t=0; t <=1.0; t+=dt)
        {
            geometry_msgs::msg::PoseStamped p = startOrientedPose;
            
            p.pose.position.x =  startOrientedPose.pose.position.x *(1 - t) + goal.x * t;
            p.pose.position.y =  startOrientedPose.pose.position.y *(1 - t) + goal.y * t;
            p.pose.orientation = startOrientedPose.pose.orientation;
            
            plan.push_back(p);
        }
    
        return plan.back();
    }
}