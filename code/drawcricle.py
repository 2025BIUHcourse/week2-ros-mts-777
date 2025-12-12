#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_circle():
    rospy.init_node('turtle_circle')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 2.0  # 线速度
        twist.angular.z = 1.5  # 角速度（形成圆周运动）
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_circle()
    except rospy.ROSInterruptException:
        pass