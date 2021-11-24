#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <sstream>
#include <bits/stdc++.h>
#include <math.h>
#include <vector>
#include <string>

using sensor_msgs::LaserScan;
using geometry_msgs::Twist;
using std::vector;
using std::string;

ros::Publisher pub;

void laserCallback(const LaserScan::ConstPtr& scan) {

  // Parsing scan data.
  ROS_INFO("LaserScan (val,angle)=(%f,%f", msg->range_min,msg->angle_min);

  // Creating Twist object.
  Twist msg;

  // Building msg.

  // Publishing next movement data
  //pub.publish(msg);
}

int main (int argc , char **argv) {

  // Initialize the ROS system and become a node.
  ros::init(argc ,argv ,"assignement_b_node");
  ros::NodeHandle nh;

  // Subscribe to laser scan output data.
  ros::Subscriber sub_laser = nh.subscribe("/scan", 1, laserCallback);

  // Create a publisher object.
  pub = nh.advertise<Twist>("/cmd_vel", 1000);

  ros::spin();
}
