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

def publisher():
	msg = Twist()
	msg.linear.x = 0.1
	msg.angular.x = 0.1
	print(msg)
	pub.publish(msg)
	
	return 0
  
  
def tester():
	msg = UInt32()
	rate = rospy.Rate(10)
	msg.data = 5
	pub.publish(msg)
	rate.sleep()
	
	
if __name__ == '__main__': 
  try:
    
    rospy.init_node('velocity_commands')
    twisty = rospy.get_param('~twisty', '/twisty') 
    
    pub = rospy.Publisher(twisty, UInt32, queue_size=1)
    
    rospy.loginfo("Node 'velocity commands' started.\nPublishing cmd_vel")
    tester()
    rospy.spin()
    
    
  except rospy.ROSInterruptException:
    pass
