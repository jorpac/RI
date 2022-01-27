#!/usr/bin/env python

import gym
import rospy
import numpy as np
import time
import math
from gym import utils, spaces
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from controllers_connection import ControllersConnection

#register the training environment in the gym as an available one
reg = register(
    id='Chair-v0',
    entry_point='chair_env:ChairEnv'
)

class ChairEnv(gym.Env):
    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment

        # gets training parameters from param server
        self.desired_distance = rospy.get_param('/desired_distance')
        self.desired_angle = rospy.get_param('/desired_angle')
        self.running_step = rospy.get_param("/running_step")

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        # self.controllers_object = ControllersConnection(namespace="chair")

        self.leadingPub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10)
        self.followerPub = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/robot2/right_front_rplidar_scan", LaserScan, self.laser_callback)
        self.scanMsg = LaserScan()
        rospy.Subscriber("/robot2/right_front_rplidar_scan", LaserScan, self.laser_callback)
        self.odomMsg = Odometry()

        self.moveLeadingChairMsg = Twist()
        self.moveLeadingChairMsg.linear.x = 30

        # Set desired world point

        """
        We consider 3 actions
        1) Move forward
        2/3) Move forward while turning left/right
        4) Do nothing
        """
        self.action_space = spaces.Discrete(4)

        self._seed()


    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        
    # Resets the state of the environment and returns an initial observation.
    def reset(self):

        # 0st: We pause the Simulator
        rospy.logdebug("Pausing SIM...")
        self.gazebo.pauseSim()

        # 1st: resets the simulation to initial values
        rospy.logdebug("Reset SIM...")
        self.gazebo.resetSim()

        # 4th: We Set the init pose to the jump topic so that the jump control can update
        # rospy.logdebug("Publish init_pose for Jump Control...>>>" + str(init_pos))
        # We check the jump publisher has connection
        # self.check_publishers_connection()

        self.leadingPub.publish(self.moveLeadingChairMsg)

        # 5th: Check all subscribers work.
        # Get the state of the Robot defined by its RPY orientation, distance from
        # desired point, contact force and JointState of the three joints
        rospy.logdebug("check_all_systems_ready...")
        # self.check_all_systems_ready()

        # 7th: pauses simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        # 8th: Get the State Discrete Stringuified version of the observations
        rospy.logdebug("get_observations...")
        state, _, _ , _ = self.get_state(self.scanMsg)

        return np.asarray(state)

    def step(self, action):

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot

        # 1st, decide which action corresponds to which joint is incremented
        cmdMsg = self.get_action_to_position(action)

        # We move it to that pos
        self.gazebo.unpauseSim()
        self.followerPub.publish(cmdMsg)
        # Then we send the command to the robot and let it go
        # for running_step seconds
        time.sleep(self.running_step)
        self.gazebo.pauseSim()
       
        state, minDist, minDistAngle, collision = self.get_state(self.scanMsg)
        reward = self.calculateReward(minDist, minDistAngle, collision)

        # odom = None
        # while odom is None:
        #     try:
        #         odom = rospy.wait_for_message('/robot2/odom', Odometry, timeout=5)
        #     except:
        #         pass

        odom = self.odomMsg

        done = False
        if odom.pose.pose.position.x > 25 or odom.pose.pose.position.y > 25:
            done = True

        return np.asarray(state), reward, collision, done

    def odom_callback(self, odomMsg):
        self.odomMsg = odomMsg

    def laser_callback(self, scanMsg):
        self.scanMsg = scanMsg


    def get_state(self, msg):
        
        # scanMsg = None
        # while scanMsg is None:
        #     try:
        #         scanMsg = rospy.wait_for_message('/robot2/right_front_rplidar_scan', LaserScan, timeout=5)
        #     except:
        #         pass

        
        ranges = self.clean_data(msg.ranges)
        #Value at 90º (left from chair)
        minDist = 9999999999
        minDistIndex = 999999999
        colliding = False
        # indexFor90Degrees = 90 / math.degrees(msg.angle_increment) # 240
        for i in ranges[240:]:
            if ranges[i] < minDist:
                minDist = ranges[i]
                minDistIndex = i
        if 0.5 > minDist > 0:
            colliding = True
        
        minDistAngle = math.degrees(msg.angle_min + minDistIndex*msg.angle_increment)
        
        return ranges + [minDist, minDistAngle], minDist, minDistAngle, colliding

    def calculateReward(self, minDist, minDistAngle, collision):
        # return -1000 if collision else 3.5 - (self_desired_distance - minDist) - (minDistAngle - 90)*0.1
        return -1000 if collision else 100 * (1 - abs(self.desired_distance - minDist) - abs(self.desired_angle - minDistAngle)*0.2)


    def get_action_to_position(self, action):

        cmdMsg = Twist()

        # catchUpCoeficient = self.minDist [0.12, 3.5] m
        if action == 0:
            # Move Forward
            cmdMsg.linear.x = 0.4
            cmdMsg.angular.z = 0
        elif action == 1:
            # Move Left
            cmdMsg.linear.x = 0.1
            cmdMsg.angular.z = math.radians(45)
        elif action == 2:
            # Move Right
            cmdMsg.linear.x = 0.1
            cmdMsg.angular.z = math.radians(-45)
        elif action == 3:
            # Do Nothing
            cmdMsg.linear.x = 0
            cmdMsg.angular.z = 0

        return cmdMsg

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self.followerPub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to followerPub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("followerPub Publisher Connected")

        rospy.logdebug("All Joint Publishers READY")

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """
        data_pose = None
        while data_pose is None and not rospy.is_shutdown():
            try:
                data_pose = rospy.wait_for_message("/odom", Odometry, timeout=0.1)
                # self.base_position = data_pose.pose.pose.position
                rospy.logdebug("Current odom READY")
            except:
                rospy.logdebug("Current odom pose not ready yet, retrying for getting robot base_position")

        laser_data = None
        while laser_data is None and not rospy.is_shutdown():
            try:
                laser_data = rospy.wait_for_message("/robot2/right_front_rplidar_scan", LaserScan, timeout=0.1)
                # Set laser scan variables
                rospy.logdebug("Current laser scan READY")
            except:
                rospy.logdebug("Current laser scan not ready yet, retrying for getting robot laser scan data")

        rospy.logdebug("ALL SYSTEMS READY")

    
    def clean_data(self, scan):
        scan_range = []
        for i in range(len(scan)):
            if scan[i] == float('Inf'):
                scan_range.append(15)
            elif np.isnan(scan[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan[i])
        return scan_range

    def close(self):
        """
        Function executed when closing the environment.
        Use it for closing GUIS and other systems that need closing.
        :return:
        """
        rospy.logdebug("Closing RobotGazeboEnvironment")
        rospy.signal_shutdown("Closing RobotGazeboEnvironment")