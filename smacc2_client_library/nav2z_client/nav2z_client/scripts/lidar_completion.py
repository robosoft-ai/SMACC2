#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import logging
import copy


class LidarProcessor(Node):
    def __init__(self):
        super().__init__("lidar_processor")

        qos_prof_1 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan, "/scan_input", self.lidar_callback, qos_profile=qos_prof_1
        )

        qos_prof_2 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        self.msg_buffer = []
        self.publisher = self.create_publisher(LaserScan, "/scan_output", qos_profile=qos_prof_2)

        self.logger = self.get_logger()
        self.logger.set_level(logging.INFO)

    def lidar_callback(self, msg):
        #self.logger.info("Received message")
        self.msg_buffer.append(msg)
        if len(self.msg_buffer) > 5:
            #self.logger.info("Merging messages")
            merged_msg = self.merge_lidar_msgs()
            #self.msg_buffer.pop(0)
            self.msg_buffer = []
            self.publisher.publish(merged_msg)  
            

    def merge_lidar_msgs(self):
        merged_ranges = [0.0] * len(self.msg_buffer[0].ranges)*len(self.msg_buffer)
        # cont_inf = 0
        cont_tot = 0
        for i in range(len(self.msg_buffer)):
            for j in range(len(self.msg_buffer[i].ranges)):
                # if self.msg_buffer[i].ranges[j] != float("inf"):
                merged_ranges[cont_tot] = self.msg_buffer[i].ranges[j]
                cont_tot += 1
                #else:
                #    cont_inf += 1

        # self.logger.info(f"Number of infinite values from total: {cont_inf}/{cont_tot}")

        merged_msg = LaserScan()
        #merged_msg = copy.deepcopy(self.msg_buffer[-1])
        merged_msg.header = self.msg_buffer[-1].header

        merged_msg.time_increment = self.msg_buffer[0].time_increment

        merged_msg.angle_increment = self.msg_buffer[0].angle_increment
        merged_msg.angle_min = self.msg_buffer[0].angle_min
        merged_msg.angle_max = self.msg_buffer[0].angle_min + (len(merged_ranges)-1)*merged_msg.angle_increment

        # print(f"first angle_min: {self.msg_buffer[0].angle_min}")
        # print(f"angle_min: {merged_msg.angle_min}")
        # print(f"angle_max: {merged_msg.angle_max}")
        
        #merged_msg.angle_min = self.msg_buffer[0].angle_min - self.msg_buffer[0].angle_increment*len(self.msg_buffer)
        #merged_msg.angle_max = merged_msg.angle_min + (len(merged_ranges)-1)*merged_msg.angle_increment

        merged_msg.range_min = min([msg.range_min for msg in self.msg_buffer])
        merged_msg.range_max = max([msg.range_max for msg in self.msg_buffer])
        merged_msg.ranges = merged_ranges

        return merged_msg


def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
