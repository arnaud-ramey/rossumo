#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author Melonee Wise

import roslib; roslib.load_manifest('rossumo')
import rospy
import std_msgs
import sensor_msgs
import geometry_msgs

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def mycallback(joy):
  global high_jump_before
  vel = geometry_msgs.msg.Twist()
  vel.linear.x = (joy.axes[axis_linear] * scale_linear);
  vel.angular.z = (joy.axes[axis_angular] * scale_angular);
  joy_pub.publish(vel)
  if (joy.buttons[button_high_jump]):
    if (high_jump_before == False):
      print("Starting high jump!")
      high_jump_before = True
      high_jump_pub.publish(std_msgs.msg.Empty())
  else:
    high_jump_before = False

if __name__ == '__main__':
  name ='sumo_teleop_joy'
  rospy.init_node(name)
  axis_linear = rospy.get_param('~axis_linear', 1)
  axis_angular = rospy.get_param('~axis_angular', 2)
  button_high_jump = rospy.get_param('~button_high_jump', 4)
  button_posture = rospy.get_param('~button_posture', 5)
  scale_linear = rospy.get_param('~scale_linear', 1.0)
  scale_angular = rospy.get_param('~scale_angular', 1.0)
  joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, mycallback)
  joy_pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
  high_jump_pub = rospy.Publisher("high_jump", std_msgs.msg.Empty, queue_size=1)
  high_jump_before = False
  rospy.spin()
