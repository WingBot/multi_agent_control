#include "multi_agent_control/agent_control.h"

Multibot::Multibot(){}
Multibot::~Multibot(){}

void Multibot::loop()
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_("~");

    nh_p_.param<std::string>("odom_frame",odom_frame_,std::string("odom"));
    nh_p_.param<std::string>("base_frame",base_frame_,std::string("base_footprint"));
    
    //agent param
    nh_p_.param<double>("dis_goal",agv_.dis_goal,2.0);
    nh_p_.param<double>("dis_safe",agv_.dis_safe,0.2);
    nh_p_.param<double>("tolerance_angle",agv_.tolerance_angle,0.5*M_PI/180);
    nh_p_.param<double>("tolerance_dis",agv_.tolerance_dis,0.01);
//     nh_p_.param<double>("dis_safe",agv_.dis_safe,0.2);
    
    //rnn param
    nh_p_.param<float>("B_k",rnn_.B_k,-2.0);
    nh_p_.param<float>("Bright_k",rnn_.Bright_k,-50.0);
    nh_p_.param<float>("lambda",rnn_.lambda,0.0);
    nh_p_.param<float>("epsilon",rnn_.epsilon,0.1);
    nh_p_.param<double>("obstacle_x",rnn_.obstacle_.x,1.0);
    nh_p_.param<double>("obstacle_y",rnn_.obstacle_.y,0.1);
    nh_p_.param<double>("vel_pid_k_x",rnn_.vel_pid_k_.x,2.0);
    nh_p_.param<double>("vel_pid_k_y",rnn_.vel_pid_k_.y,2.0);
    nh_p_.param<int>("control_rate",rnn_.control_rate,20);
    
    ROS_INFO_STREAM("odom_frame_: " << odom_frame_);
    ROS_INFO_STREAM("base_frame_: " << base_frame_);
    
    if(initMultibot())
    {

        cmd_pub_  = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",10,&Multibot::odom_callback,this);
        ros::Timer send_speed_timer = nh_.createTimer(ros::Duration(1.0/rnn_.control_rate),&Multibot::send_speed_callback,this);
//         SingleAgentControl();
        ros::spin();
        return;
    }
}


bool Multibot::initMultibot()
{
    try{
            listener_.waitForTransform(odom_frame_, base_frame_, ros::Time(0), ros::Duration(5.0));
            listener_.lookupTransform(odom_frame_, base_frame_,  ros::Time(0), transform_);
        }
    catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
//             tf_error_ = true;
            return false;
        }
    agv_.pos_start_.x = transform_.getOrigin().x();
    agv_.pos_start_.y = transform_.getOrigin().y();
    agv_.pos_start_.z = 0.0;
//     agv_.pos_start_.z = acos(transform.getRotation().z())*2;
    agv_.pos_feedback_.x = transform_.getOrigin().x();
    agv_.pos_feedback_.y = transform_.getOrigin().y();


    rnn_.B_.x = 0.0;
    rnn_.B_.y = 0.0;
    rnn_.Bright = 0.0;
    
    t_start = ros::Time::now();
    
    ROS_INFO_STREAM_ONCE("x_start = "<< agv_.pos_start_.x);
    ROS_INFO_STREAM_ONCE("y_start = "<< agv_.pos_start_.y);
    ROS_INFO_STREAM_ONCE("0_start = "<< agv_.pos_start_.z);
    
    return true;
}


void Multibot::send_speed_callback(const ros::TimerEvent&)
{
        ROS_INFO_STREAM_ONCE("Entering SingleAgentControl.........");
        agv_.pos_obstacle_.x = agv_.pos_start_.x + rnn_.obstacle_.x;
        agv_.pos_obstacle_.y = agv_.pos_start_.y + rnn_.obstacle_.y;
        agv_.dis_delta_goal = agv_.dis_goal;
        agv_.pos_goal_.x = agv_.pos_start_.x + agv_.dis_goal;
        agv_.pos_goal_.y = agv_.pos_start_.y;
        agv_.pos_desired_.x = agv_.pos_start_.x;
        agv_.pos_desired_.y = agv_.pos_start_.y;
//         ROS_INFO_STREAM("---agv.vel_x: " << agv_.vel_feedback_.x); 
//         ROS_INFO_STREAM("---agv.vel_y: " << agv_.vel_feedback_.y);
        
}


void Multibot::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
     ROS_INFO_STREAM_ONCE("Entering odom_callback.........");
     odom_resv_ = *odom_msg;
     agv_.vel_feedback_.x = odom_resv_.twist.twist.linear.x;
     agv_.vel_feedback_.y = odom_resv_.twist.twist.linear.y;
     
     ROS_INFO_STREAM_ONCE("odom_frame_recv: " << odom_resv_.header.frame_id);
//      ROS_INFO_STREAM("agv_.vel_feedback_.x: " << agv_.vel_feedback_.x); 
//      ROS_INFO_STREAM("agv_.vel_feedback_.y: " << agv_.vel_feedback_.y);
}
