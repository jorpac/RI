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
        rospy.loginfo('{} \n was the msg received'.format(scanMsg.ranges.size()))
        newMoveMsg = Twist()
        comp_distance = 999999999
        min_range_index = 0
        i = 0
        for distance in  scanMsg.ranges:
            if (distance < comp_distance):
                min_range_index = i
                comp_distance = distance
            i=i+1

        newMoveMsg.linear.x = 0.4
        newMoveMsg.angular.z = 0
        angle = (scanMsg.angle_min * (180 / math.pi)) + (scanMsg.angle_increment * (180 / math.pi)) * min_range_index
        alpha = 90.0 - abs(angle)

        if (comp_distance <= scanMsg.range_max):
            newMoveMsg.angular.z = (-40 * (math.sin(alpha * (math.pi / 180)) - (comp_distance - 0.4))) * 0.1

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

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        # pub2.publish(cmdMsg)
        pub1.publish(goalMsg)
        pub3.publish(hello_str)
        rate.sleep() 

if __name__ == '__main__':
    main()