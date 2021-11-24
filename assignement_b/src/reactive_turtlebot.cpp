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

class MoveAroundB {
  public:
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::NodeHandle nh;
    geometry_msgs::Twist msg_;
    
    void abc() {
      pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
      sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &MoveAroundB::counterCallback, this);
      ros::spin();
    }
  
  void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    float angle = 0.0;
	  float comp_distance = 999999999;
    for (unsigned int i = 0; i < msg->ranges.size(); i++) {
      float distance = msg->ranges[i];
      float sensorAngle = (msg->angle_min * (180 / PI)) + (msg->angle_increment * (180 / PI)) * i;
      if (distance < comp_distance) {
        angle = sensorAngle;
        comp_distance = distance;
      }
    }
    msg_.linear.x = 0.2;
    msg_.angular.z = 0;
    float alpha = 90.0 - abs(angle);

    if (comp_distance <= msg->range_max)
		//theory here: https://www.seas.upenn.edu/sunfest/docs/papers/12-bayer.pdf adapted from formula at page 6
		msg_.angular.z = (-20 * (sin(alpha * (PI / 180)) - (comp_distance - 0.4))) * 0.1;
    pub.publish(msg_);
  }
};

int main (int argc , char **argv) {
  // Initialize the ROS system and become a node .
  ros::init(argc ,argv ,"assignement_b_node");

  MoveAroundB move;
  move.abc();
  ros::spin();

}
