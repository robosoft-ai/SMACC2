// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

// std::string newFrame;
// std::string orgFrame;

rclcpp::Node::SharedPtr nh ;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr repub_odom ;

void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
  if ((msg->pose.pose.position.x != 0) &&
      (msg->pose.pose.position.y != 0) &&
      (msg->pose.pose.position.z != 0)) {
    //static tf::TransformBroadcaster br;
    static tf2_ros::TransformBroadcaster br(nh);
    geometry_msgs::msg::TransformStamped transformStamped;

    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
    //                                 msg->pose.pose.position.y,
    //                                 msg->pose.pose.position.z));
    // tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x,
    //          msg->pose.pose.orientation.y,
    //          msg->pose.pose.orientation.z,
    //          msg->pose.pose.orientation.w);
    // transform.setRotation(q);

    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = "odom";

    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    // tf2::Quaternion q;
    // q.setRPY(0, 0, msg->theta);
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

    br.sendTransform(transformStamped);
    repub_odom->publish(*msg);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("odom_to_tf");

  std::string odomTopic = "odom";
  std::string odomTopicRepub = "odom_out";

//   nh.param<std::string>("odomTopic", odomTopic, "/Base/neighbors/H01/odometry");
//   nh.param<std::string>("newFrame", newFrame, "H01_base/base_link");
//   nh.param<std::string>("orgFrame", orgFrame, "world");
  auto sub_odom = nh->create_subscription<nav_msgs::msg::Odometry>(odomTopic, rclcpp::SensorDataQoS(), &odomCallback);
  repub_odom = nh->create_publisher<nav_msgs::msg::Odometry>(odomTopicRepub, 10 );

  rclcpp::spin(nh);
  return 0;
}
