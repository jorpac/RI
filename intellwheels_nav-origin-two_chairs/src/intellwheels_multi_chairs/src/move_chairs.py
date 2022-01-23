#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

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

def main():
    rospy.init_node('move_chairs')
    leadPublisher = rospy.Publisher("/robot1/move_base_simple/goal", PoseStamped, queue_size=10)
    followPublisher = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=10)

    c1 = LeadingChair(leadPublisher)
    c2 = FollowerChair(followPublisher)

    rospy.Subscriber("/robot2/right_front_rplidar_scan", LaserScan, c2.follow)
    rospy.spin()

    c1.goToGoal((1, 2))

if __name__ == '__main__':
    main()
