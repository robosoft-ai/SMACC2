#!/usr/bin/env python3
# Copyright 2026 RobosoftAI Inc.
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

"""Accumulate the flown lidar cloud into a voxel map and save it for offline analysis.

Subscribes to the bridged PointCloud2, transforms every Nth cloud into the map
frame with TF (at the cloud's stamp), keeps the occupied voxels (default 0.25 m)
in a set and writes them as an Nx3 float32 array (voxel centres, map frame,
ENU) to `output` every `save_period` seconds and on shutdown.

  ros2 run sm_cl_px4_mr_test_5 cloud_map_recorder.py --ros-args -p output:=/tmp/map.npy
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import transforms3d.quaternions as tq


class CloudMapRecorder(Node):
    def __init__(self):
        super().__init__("cloud_map_recorder")
        self.declare_parameter("topic", "/lidar/points")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("output", "/tmp/sm_cl_px4_mr_test_5_map.npy")
        self.declare_parameter("voxel_m", 0.25)
        self.declare_parameter("every_nth", 2)
        self.declare_parameter("max_range_m", 30.0)
        self.declare_parameter("save_period_s", 30.0)
        self.topic = self.get_parameter("topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.output = self.get_parameter("output").value
        self.voxel = float(self.get_parameter("voxel_m").value)
        self.every_nth = int(self.get_parameter("every_nth").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.save_period = float(self.get_parameter("save_period_s").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.voxels = set()
        self.count = 0
        self.used = 0
        self.last_save = time.monotonic()
        self.sub = self.create_subscription(PointCloud2, self.topic, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f"recording {self.topic} into {self.output} (voxel {self.voxel} m, every {self.every_nth}th cloud)")

    def on_cloud(self, msg: PointCloud2):
        self.count += 1
        if self.count % self.every_nth:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, msg.header.stamp,
                                                 timeout=rclpy.duration.Duration(seconds=0.2))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if self.used == 0 and self.count % 50 == 0:
                self.get_logger().warn(f"no TF {self.map_frame} <- {msg.header.frame_id} yet: {e}")
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if pts.size == 0:
            return
        pts = pts.astype(np.float64)
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > 0.5) & (r < self.max_range)]
        q = tf.transform.rotation
        rot = tq.quat2mat([q.w, q.x, q.y, q.z])
        t = tf.transform.translation
        world = pts @ rot.T + np.array([t.x, t.y, t.z])
        idx = np.floor(world / self.voxel).astype(np.int32)
        self.voxels.update(map(tuple, np.unique(idx, axis=0)))
        self.used += 1
        if time.monotonic() - self.last_save > self.save_period:
            self.save()

    def save(self):
        if not self.voxels:
            return
        arr = (np.array(sorted(self.voxels), dtype=np.float32) + 0.5) * self.voxel
        np.save(self.output, arr)
        self.last_save = time.monotonic()
        self.get_logger().info(f"saved {len(arr)} voxels from {self.used} clouds to {self.output}")


def main():
    rclpy.init()
    node = CloudMapRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
