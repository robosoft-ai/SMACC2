#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy
import std_msgs
from std_msgs.msg import UInt16
import std_srvs
from std_srvs.srv import SetBool


if __name__=="__main__":
  def set_bool_service(req):
      rospy.loginfo("RECEIVING SET BOOL SERVICE REQUEST: value="+ str(req.data))

      response = std_srvs.srv.SetBoolResponse()
      response.message = "OK, value set"
      response.success = True
      return response

  rospy.init_node('service_node3')
  s = rospy.Service('service_node3', std_srvs.srv.SetBool, set_bool_service)
  rospy.spin()