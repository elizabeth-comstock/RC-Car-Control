#!/usr/bin/env python
# license removed for brevity
import rospy, math
from std_msgs.msg import UInt32
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
		msg = Twist()
		msg.linear.x = 0.02
		msg.angular.z = 0.1
		#rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
