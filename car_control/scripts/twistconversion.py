#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  twistconversion.py
#  
#  Copyright 2018 qiheng <qiheng@qiheng-UX430UQ>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

import rospy, math
from geometry_msgs.msg import Twist
#from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import UInt32
global ackermann

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase): #radians
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)

def convert_steering(steering,scale):
	if steering == 0:
		return 0
		
	max_angle = radians(60)
	return (steering/max_angle)*scale + 90
	
def convert_throttle(v, scale):
	if v == 0:
		return 0
	max_v = 0.20
	return (v/max_v)*scale + 100
	

def cmd_callback(data):
  global wheelbase
  global frame_id
  global pub
  
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  msg.data = 1000*convert_steering(steering, 20) + convert_throttle(v,15)
  
  
  #msg = AckermannDriveStamped()
  #msg.header.stamp = rospy.Time.now()
  #msg.header.frame_id = frame_id
  #msg.drive.steering_angle = steering
  #msg.drive.speed = v
  
  
  pub.publish(msg)
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_arduino')
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
#   ackermann = rospy.get_param('~ackermann', '/ackermann')
#   ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    wheelbase = rospy.get_param('~wheelbase', 0.6)
    frame_id = rospy.get_param('~frame_id', 'camera/odom')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(ackermann, Uint32, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_arduino' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
