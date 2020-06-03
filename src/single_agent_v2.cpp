#include "ros/ros.h"
#include "std_msgs/String.h"//geometry_msgs
#include "geometry_msgs/Twist.h"//包含velocity space消息
#include <tf/transform_listener.h>
#include "math.h"
#include <sstream>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

// ros::Publisher odom_pub;
nav_msgs::Odometry odom_resv_;
double v_x_t = 0.0;//向前的线速度0.2m/s
double v_y_t = 0.0;//向前的线速度0.2m/s
double odom_x = 0.0;
double odom_y = 0.0;
double odom_theta = 0.0;
void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
//     ROS_INFO_STREAM("OdomCallback_________ = ");
    odom_resv_ = *odom_msg;
    v_x_t = odom_resv_.twist.twist.linear.x;
    v_y_t = odom_resv_.twist.twist.linear.y;
    odom_x = odom_resv_.pose.pose.position.x;
    odom_y  = odom_resv_.pose.pose.position.y;
//     odom_theta = 
    ROS_INFO_STREAM("OdomCallback_________v_x_t = "<< v_x_t);
    ROS_INFO_STREAM("OdomCallback_________v_y_t = "<< v_y_t);
    ROS_INFO_STREAM("OdomCallback_________odom_x = "<< odom_x);
    ROS_INFO_STREAM("OdomCallback_________odom_y = "<< odom_y);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "single_agent");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry> ("/odom", 5, OdomCallback);
//   odom_pub = nh.advertise<nav_msgs::Odometry> ("/odom", 5);//;
  ros::spin ();
}
