/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
// #include <rclcpp/rclcpp.hpp>

#include <angles/angles.h>
#include <move_base_z_planners_common/common.h>
#include <move_base_z_planners_common/move_base_z_client_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace cl_move_base_z
{
geometry_msgs::msg::PoseStamped makePureSpinningSubPlan(const geometry_msgs::msg::PoseStamped& start, double dstRads,
                                                        std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                                        double radstep)
{
  double startYaw = tf2::getYaw(start.pose.orientation);
  // RCLCPP_INFO(getNode()->get_logger(),"pure spining start yaw: %lf", startYaw);
  // RCLCPP_INFO(getNode()->get_logger(),"pure spining goal yaw: %lf", dstRads);
  // RCLCPP_WARN_STREAM(getNode()->get_logger(),"pure spinning start pose: " << start);

  double goalAngleOffset = angles::shortest_angular_distance(startYaw, dstRads);
  // RCLCPP_INFO(getNode()->get_logger(),"shortest angle: %lf", goalAngleOffset);


  if (goalAngleOffset >= 0)
  {
    // angle positive turn counterclockwise
    // RCLCPP_INFO(getNode()->get_logger(),"pure spining counterclockwise");
    for (double dangle = 0; dangle <= goalAngleOffset; dangle += radstep)
    {
      geometry_msgs::msg::PoseStamped p = start;
      double yaw = startYaw + dangle;
      // RCLCPP_INFO(getNode()->get_logger(),"pure spining counterclockwise, current path yaw: %lf, dangle: %lf,
      // angleoffset %lf, radstep %lf pathsize(%ld)", yaw, dangle, goalAngleOffset, radstep, plan.size());
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      p.pose.orientation = tf2::toMsg(q);
      plan.push_back(p);
    }
  }
  else
  {
    // angle positive turn clockwise
    // RCLCPP_INFO(getNode()->get_logger(),"pure spining clockwise");
    for (double dangle = 0; dangle >= goalAngleOffset; dangle -= radstep)
    {
      // RCLCPP_INFO(getNode()->get_logger(),"dangle: %lf", dangle);
      geometry_msgs::msg::PoseStamped p = start;
      double yaw = startYaw + dangle;
      // RCLCPP_INFO(getNode()->get_logger(),"pure spining clockwise, yaw: %lf, dangle: %lf, angleoffset %lf radstep
      // %lf", yaw, dangle, goalAngleOffset,radstep);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      p.pose.orientation = tf2::toMsg(q);
      plan.push_back(p);
    }
  }

  // RCLCPP_INFO(getNode()->get_logger(),"pure spining end yaw: %lf", dstRads);
  geometry_msgs::msg::PoseStamped end = start;
  tf2::Quaternion q;
  q.setRPY( 0, 0, dstRads);
  end.pose.orientation = tf2::toMsg(q);
  plan.push_back(end);

  return end;
}

geometry_msgs::msg::PoseStamped makePureStraightSubPlan(const geometry_msgs::msg::PoseStamped& startOrientedPose,
                                                        const geometry_msgs::msg::Point& goal, double lenght,
                                                        std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  double dx = 0.01;  // 1 cm
  double steps = lenght / dx;
  double dt = 1.0 / steps;

  // geometry_msgs::msg::PoseStamped end;
  // end.pose.orientation = startOrientedPose.pose.orientation;
  // end.pose.position = goal;
  plan.push_back(startOrientedPose);

  // RCLCPP_INFO_STREAM(nh_->get_logger(),"Pure straight, start: " << startOrientedPose << std::endl << "end: " <<
  // goal);
  for (double t = 0; t <= 1.0; t += dt)
  {
    geometry_msgs::msg::PoseStamped p = startOrientedPose;

    p.pose.position.x = startOrientedPose.pose.position.x * (1 - t) + goal.x * t;
    p.pose.position.y = startOrientedPose.pose.position.y * (1 - t) + goal.y * t;
    p.pose.orientation = startOrientedPose.pose.orientation;

    plan.push_back(p);
  }

  return plan.back();
}

}  // namespace cl_move_base_z

std::ostream& operator<<(std::ostream& out, const geometry_msgs::msg::Twist& msg)
{
  out << "Twist [" << msg.linear.x << "m , " << msg.linear.y << "m , " << msg.angular.z << "rad ]";
  return out;
}

std::ostream& operator<<(std::ostream& out, const geometry_msgs::msg::Pose& msg)
{
  out << "Position [" << msg.position.x << "m , " << msg.position.y << "m , " << msg.position.z << "m ]";
  out << " Orientation [" << msg.orientation.x << " , " << msg.orientation.y << " , " << msg.orientation.z << ", "
      << msg.orientation.w << "]";
  return out;
}

std::ostream& operator<<(std::ostream& out, const geometry_msgs::msg::Point& msg)
{
  out << "Position [" << msg.x << "m , " << msg.y << "m , " << msg.z << "m ]";
  return out;
}

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Quaternion &msg)
{
  out << "Quaternion [" << msg.x << " , " << msg.y << " , " << msg.z << ", " << msg.w << "]";
  return out;
}

std::ostream& operator<<(std::ostream& out, const geometry_msgs::msg::PoseStamped& msg)
{
  out << msg.pose;
  return out;
}
