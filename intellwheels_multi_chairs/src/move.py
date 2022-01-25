#!/usr/bin/env python

import math
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class LeadingChair(object):
    def __init__(self, pub):
        self.pub = pub

    def goToGoal(self, goal):
        x, y = goal # is this correct?
        goalMsg = PoseStamped()
        # alter goalMsg based on goal received
        self.pub.publish(goalMsg)

class FollowerChair(object):
    def __init__(self, pub):
        self.pub = pub

    def follow(self, scanMsg):
        rospy.loginfo('{} \n was the msg received'.format(scanMsg))
        newMoveMsg = Twist()
        # alter according to how best move the chair next to follow the one in front/to the side based on scanMsg
        self.pub.publish(newMoveMsg)

# def print(twistMsg):
#     rospy.loginfo()

def main():
    pub1 = rospy.Publisher("/robot1/move_base_simple/goal", PoseStamped, queue_size=10)
    pub2 = rospy.Publisher("/robot2/cmd_vel/", Twist, queue_size=10)
    pub3 = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('move_chairs')

    rate = rospy.Rate(10)

    cmdMsg = Twist()
    # cmdMsg.linear.y = -2
    goalMsg = PoseStamped()
    goalMsg.header.frame_id = "map"
    goalMsg.pose.position.x = 30
    goalMsg.pose.position.y = 30
    goalMsg.pose.orientation = [0, 1, 0, 0]
    goalMsg.header.stamp = rospy.Time.now()
    # Save current time and set publish rate at 10 Hz
    pub2.publish(cmdMsg)

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        # pub2.publish(cmdMsg)
        pub1.publish(goalMsg)
        pub3.publish(hello_str)
        rate.sleep() 
    # c1.goToGoal((1, 2))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass