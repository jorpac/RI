// This program pub l ishes randomly−generated v e l o c i t y
// messages for tu r t les im .

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs : : Twist
#include <stdlib.h>
#include <sstream>
#include <bits/stdc++.h>
#include <math.h>
#define PI 3.14159265358979323846

using geometry_msgs::Twist;
using sensor_msgs::LaserScan;

class MoveAroundB {
  
  public:
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::NodeHandle nh;
    Twist cmd_msg;
    
    void abc() {
      pub = nh.advertise<Twist>("/robot2/cmd_vel", 1);
      sub = nh.subscribe<LaserScan>("/robot2/scan", 1, &MoveAroundB::counterCallback, this);
      ros::spin();
    }
  
  void counterCallback(const LaserScan::ConstPtr& scan_msg){

	  float comp_distance = 999999999;
    unsigned int min_range_index = 0;

    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++) {

      float distance = scan_msg->ranges[i];

      if (distance < comp_distance) {
        min_range_index = i;
        comp_distance = distance;
      }
    }

    cmd_msg.linear.x = 0.2;
    cmd_msg.angular.z = 0;
    float angle = (scan_msg->angle_min * (180 / PI)) + (scan_msg->angle_increment * (180 / PI)) * min_range_index;
    float alpha = 90.0 - abs(angle);

    if (comp_distance <= scan_msg->range_max)
      cmd_msg.angular.z = (-20 * (sin(alpha * (PI / 180)) - (comp_distance - 0.4))) * 0.1;
    
    pub.publish(cmd_msg);
  }
};

int main (int argc , char **argv) {

  // Initialize the ROS system and become a node .
  ros::init(argc ,argv ,"assignement_b_node");

  MoveAroundB move;
  move.abc();
  ros::spin();
}
