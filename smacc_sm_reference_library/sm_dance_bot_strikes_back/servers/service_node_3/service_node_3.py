#!/usr/bin/env python3

from __future__ import print_function

#import roslib
#import rospy
import rclpy
from rclpy.node import Node

import std_msgs
from std_msgs.msg import UInt16
import std_srvs
from std_srvs.srv import SetBool

if __name__=="__main__":
  class Service3(Node):
    def __init__(self):
      super().__init__('Service3')
      self.s = self.create_service( std_srvs.srv.SetBool, 'service_node3', self.set_bool_service)

    def set_bool_service(self, req, res):    
        self.get_logger().info("RECEIVING SET BOOL SERVICE REQUEST: value="+ str(req.data))
        
        res.message = "OK, value set"
        res.success = True
        return res

  rclpy.init(args=None)
  s = Service3()
  
  #s = rospy.Service('service_node3', std_srvs.srv.SetBool, set_bool_service)
  #rospy.spin()
  rclpy.spin(s)
  rclpy.shutdown()