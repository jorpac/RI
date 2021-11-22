// This program pub l ishes randomly−generated v e l o c i t y
// messages for tu r t les im .

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs : : Twist
#include <stdlib.h>
#include <sstream>
#include <bits/stdc++.h>
#include <math.h>

int main (int argc , char **argv) {
  // Initialize the ROS system and become a node .
  ros::init(argc ,argv ,"assignement_b_node");
  ros::NodeHandle nh;
  // Create a publisher object .
  ros::Publisher pub = nh.advertise <geometry_msgs::Twist >("turtlebot", 1000);
  // Seed the random number generator .
  srand(time(0));
  // Loop at 2Hz until the node is shut down .
  double i = 0.2;
  ros::Rate rate(2);
  while (ros::ok()){
  // Create and fill in the message. The other four
  // fields, which are ignored by turt lesim , defaultto 0.
  geometry_msgs::Twist msg;

  double(rand())/double(RAND_MAX);

  msg.linear.x = double(i*(i/i-0.1));
  msg.angular.z = 100;
  i = i + 0.1;
  // Pub lish the message .
  pub.publish(msg);
  // Send a message to rosout with the d e t a i l s .
  ROS_INFO_STREAM( " Sending ␣random␣velocity␣command : " << "␣linear=" << msg.linear.x << "␣angular=" << msg.angular.z);
  // Wait un t i l i t ' s time for another i t e ra t ion .
  rate.sleep();
  }
}
