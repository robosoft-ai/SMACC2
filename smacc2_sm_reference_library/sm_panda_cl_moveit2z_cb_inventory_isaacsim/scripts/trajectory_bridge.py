#!/usr/bin/env python3
# Copyright 2025 Robosoft Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Trajectory Bridge Node for Isaac Sim + MoveIt2 Integration

This node bridges MoveIt2's FollowJointTrajectory action to Isaac Sim's
joint command topics. It provides an action server that MoveIt2 can connect to,
and publishes joint commands that Isaac Sim's OmniGraph can subscribe to.

Usage:
    ros2 run sm_panda_cl_moveit2z_cb_inventory_isaacsim trajectory_bridge.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import threading
import time


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__("trajectory_bridge")

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter("joint_command_topic", "joint_commands")
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("action_name", "panda_arm_controller/follow_joint_trajectory")
        self.declare_parameter("position_tolerance", 0.01)  # radians
        self.declare_parameter("goal_time_tolerance", 1.0)  # seconds

        joint_command_topic = self.get_parameter("joint_command_topic").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        action_name = self.get_parameter("action_name").value

        # Publisher for joint commands to Isaac Sim
        self.joint_command_pub = self.create_publisher(JointState, joint_command_topic, 10)

        # Subscriber for joint states from Isaac Sim
        self.current_joint_state = None
        self.joint_state_lock = threading.Lock()
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        # Action server for MoveIt2
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self.execute_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(f"Trajectory Bridge started")
        self.get_logger().info(f"  Action server: {action_name}")
        self.get_logger().info(f"  Publishing commands to: {joint_command_topic}")
        self.get_logger().info(f"  Subscribing to states from: {joint_state_topic}")

    def joint_state_callback(self, msg):
        with self.joint_state_lock:
            self.current_joint_state = msg

    def get_current_joint_state(self):
        with self.joint_state_lock:
            return self.current_joint_state

    def goal_callback(self, goal_request):
        """Accept all trajectory goals"""
        self.get_logger().info("Received trajectory goal request - ACCEPTING")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept all cancel requests"""
        self.get_logger().info("Received cancel request - ACCEPTING")
        return CancelResponse.ACCEPT

    def execute_trajectory(self, goal_handle):
        """Execute the trajectory received from MoveIt2"""
        self.get_logger().info("Received trajectory goal")

        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points

        if len(points) == 0:
            self.get_logger().warn("Empty trajectory received")
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        self.get_logger().info(
            f"Executing trajectory with {len(points)} points for joints: {joint_names}"
        )

        # Get trajectory duration
        last_point = points[-1]
        total_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
        self.get_logger().info(f"Trajectory duration: {total_duration:.2f} seconds")

        # Publish rate (Hz) - Isaac Sim needs frequent updates
        publish_rate = 50.0  # 50 Hz
        dt = 1.0 / publish_rate

        # Get start time
        start_time = self.get_clock().now()
        feedback = FollowJointTrajectory.Feedback()

        # Find point times for interpolation
        point_times = []
        for p in points:
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            point_times.append(t)

        current_point_idx = 0

        while True:
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Trajectory canceled")
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result

            # Calculate elapsed time
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9

            # Check if trajectory is complete
            if elapsed >= total_duration:
                break

            # Find the surrounding trajectory points for interpolation
            while (
                current_point_idx < len(points) - 1
                and point_times[current_point_idx + 1] <= elapsed
            ):
                current_point_idx += 1

            # Get target position (interpolate if between points)
            if current_point_idx >= len(points) - 1:
                # Use last point
                target_positions = list(points[-1].positions)
            else:
                # Linear interpolation between points
                p0 = points[current_point_idx]
                p1 = points[current_point_idx + 1]
                t0 = point_times[current_point_idx]
                t1 = point_times[current_point_idx + 1]

                if t1 > t0:
                    alpha = (elapsed - t0) / (t1 - t0)
                    alpha = max(0.0, min(1.0, alpha))
                else:
                    alpha = 1.0

                target_positions = []
                for j in range(len(joint_names)):
                    pos0 = p0.positions[j] if j < len(p0.positions) else 0.0
                    pos1 = p1.positions[j] if j < len(p1.positions) else 0.0
                    target_positions.append(pos0 + alpha * (pos1 - pos0))

            # Publish joint command
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = joint_names
            cmd.position = target_positions

            self.joint_command_pub.publish(cmd)

            # Publish feedback periodically
            if int(elapsed * 10) % 5 == 0:  # Every 0.5 seconds
                feedback.joint_names = joint_names
                desired_point = JointTrajectoryPoint()
                desired_point.positions = target_positions
                feedback.desired = desired_point

                current = self.get_current_joint_state()
                if current:
                    actual_point = JointTrajectoryPoint()
                    actual_positions = []
                    for name in joint_names:
                        if name in current.name:
                            idx = current.name.index(name)
                            actual_positions.append(current.position[idx])
                        else:
                            actual_positions.append(0.0)
                    actual_point.positions = actual_positions
                    feedback.actual = actual_point

                goal_handle.publish_feedback(feedback)

            # Sleep to maintain publish rate
            time.sleep(dt)

        # Send final position a few more times to ensure it's received
        final_positions = list(points[-1].positions)
        for _ in range(10):
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = joint_names
            cmd.position = final_positions
            self.joint_command_pub.publish(cmd)
            time.sleep(0.02)

        self.get_logger().info("Trajectory execution completed")
        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("Trajectory execution completed successfully")

        return result


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
