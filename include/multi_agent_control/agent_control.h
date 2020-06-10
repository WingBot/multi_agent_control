/*
 *      Author: szr@giimagv
 */

#ifndef INCLUDE_AGENT_CONTROL_H_
#define INCLUDE_AGENT_CONTROL_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

struct double_xyz
{
    double x;
    double y;
    double z;
};

struct double_xy
{
    double x;
    double y;
};

struct RNN_param
{
    double_xy B_;
    double Bright;
    float B_k;
    float Bright_k;
    float lambda;
    float epsilon;
    double_xyz vel_pid_k_;
    double_xy obstacle_;
    int control_rate;
};

struct agent_param
{
    double tolerance_angle;
    double tolerance_dis;
    double_xyz vel_feedback_;
    double_xyz vel_desired_;
    double_xyz vel_limit_;
    double dis_safe;
    double dis_goal;
    double dis_delta_goal;
    double dis_delta_obstacle;
    double_xyz pos_start_;
    double_xy pos_feedback_;
    double_xy pos_goal_;
    double_xy pos_desired_;
    double_xy pos_obstacle_;
};

class Multibot
{
    public:
        Multibot();
        ~Multibot();
        void loop();
    
    private:
        bool initMultibot();
        void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
//         void AgentControl();
        void send_speed_callback(const ros::TimerEvent&);
        
        nav_msgs::Odometry odom_resv_;
        geometry_msgs::Twist move_cmd_;
//         double v_x_t;
//         double v_y_t;
        geometry_msgs::Twist vel_t_;
        
        ros::Subscriber odom_sub_;
        ros::Publisher cmd_pub_;
                
        std::string odom_frame_;
        std::string base_frame_;

        tf::TransformListener listener_;
        tf::StampedTransform transform_;
//         bool tf_error_;
        
        ros::Time t_run_;
        ros::Time t_last_;
        ros::Time t_start;
        

        

        //agent param
        struct agent_param agv_;
//         double dis_goal;
//         double dis_safe;
//         double tolerance_angle;
//         double tolerance_dis;
        
        //rnn param
        struct RNN_param rnn_;
//         float B_k;
//         float Bright_k;
//         float lambda;
//         float epsilon;
//         double v_x_t;
        double_xy obstacle_; 
        
};

#endif /* INCLUDE_AGENT_CONTROL_H_ */
