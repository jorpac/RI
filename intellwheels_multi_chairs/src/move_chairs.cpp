#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>
#include <sstream>
#include <bits/stdc++.h>
#include <math.h>
#define PI 3.14159265358979323846

using namespace std;
using geometry_msgs::Twist;
using geometry_msgs::PoseStamped;
using sensor_msgs::LaserScan;

class Chair1 {

  public:
    ros::NodeHandle nh;
    // ros::Subscriber sub;
    ros::Publisher pub = nh.advertise<PoseStamped>("/robot1/move_base_simple/goal", 1);
    PoseStamped goal_msg;

    void goToGoal() {

      pub.publish(goal_msg);
    }
};

class Chair2 {

  public:
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<Twist>("/robot2/cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe<LaserScan>("/robot2/right_front_rplidar_scan", 1, &Chair2::recalculateFollowPath, this);
    Twist cmd_msg;

    void move() {
        
        ros::spin();
    }
  
  void recalculateFollowPath(const LaserScan::ConstPtr& scan_msg){

	  
    
    pub.publish(cmd_msg);
  }
};

int main (int argc , char **argv) {

  // Initialize the ROS system and become a node .
  ros::init(argc ,argv ,"assignement_b_node");

  Chair1 chair1;
  chair1.goToGoal();
  Chair2 chair2;
  chair2.move();
  ros::spin();
}
