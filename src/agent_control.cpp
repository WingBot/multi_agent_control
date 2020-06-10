#include "multi_agent_control/agent_control.h"

Multibot::Multibot():tf_error_(false){}
Multibot::~Multibot(){}

void Multibot::loop()
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_("~");

    nh_p_.param<std::string>("odom_frame",odom_frame_,std::string("odom"));
    nh_p_.param<std::string>("base_frame",base_frame_,std::string("base_footprint"));
    
    //agent param
    nh_p_.param<double>("dis_goal",dis_goal,2.0);
    nh_p_.param<double>("dis_safe",dis_safe,0.2);
    nh_p_.param<double>("tolerance_angle",tolerance_angle,0.5*M_PI/180);
    nh_p_.param<double>("tolerance_dis",tolerance_dis,0.01);
    nh_p_.param<double>("vel_desired_x",vel_desired_.x,0.1);
    nh_p_.param<double>("vel_desired_y",vel_desired_.y,0.0);
    nh_p_.param<double>("vel_limit_x",vel_limit_.x,0.2);
    nh_p_.param<double>("vel_limit_y",vel_limit_.y,0.2);
//     nh_p_.param<double>("dis_safe", dis_safe,0.2);
    
    //rnn param
    nh_p_.param<float>("B_k",B_k,-2.0);
    nh_p_.param<float>("Bright_k",Bright_k,-50.0);
    nh_p_.param<float>("lambda",lambda,0.0);
    nh_p_.param<float>("epsilon",epsilon,0.1);
    nh_p_.param<float>("epsilon_k",epsilon_k,2.0);
    nh_p_.param<double>("obstacle_x",obstacle_.x,1.0);
    nh_p_.param<double>("obstacle_y",obstacle_.y,0.1);
    nh_p_.param<double>("vel_pid_k_x",vel_pid_k_.x,2.0);
    nh_p_.param<double>("vel_pid_k_y",vel_pid_k_.y,2.0);
    nh_p_.param<int>("control_rate",control_rate,20);
    
    ROS_INFO_STREAM("odom_frame_: " << odom_frame_);
    ROS_INFO_STREAM("base_frame_: " << base_frame_);
    
    if(initMultibot())
    {

        cmd_pub_  = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",10,&Multibot::odom_callback,this);
        delta_dis_pub_  = nh_.advertise<geometry_msgs::PoseStamped>("rnn_output",10);
        ros::Timer send_speed_timer = nh_.createTimer(ros::Duration(1.0/control_rate),&Multibot::send_speed_callback,this);
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
            return false;
        }
    pos_start_.x = transform_.getOrigin().x();
    pos_start_.y = transform_.getOrigin().y();
    pos_start_.z = 0.0;
//      pos_start_.z = acos(transform.getRotation().z())*2;
    pos_feedback_.x = transform_.getOrigin().x();
    pos_feedback_.y = transform_.getOrigin().y();


//      B_.x = 0.0;
//      B_.y = 0.0;
//      Bright = 0.0;
    
    t_start_ = ros::Time::now();
    move_cmd_.linear.x=move_cmd_.linear.y=move_cmd_.linear.z=0;
    move_cmd_.angular.x=move_cmd_.angular.y=move_cmd_.angular.z=0;
    delta_dis_.pose.position.x = delta_dis_.pose.position.y = delta_dis_.pose.position.z = 0.0;
    
    ROS_INFO_STREAM_ONCE("x_start = "<< pos_start_.x);
    ROS_INFO_STREAM_ONCE("y_start = "<< pos_start_.y);
    ROS_INFO_STREAM_ONCE("0_start = "<< pos_start_.z);
    
    return true;
}


void Multibot::send_speed_callback(const ros::TimerEvent&)
{
        ROS_INFO_STREAM_ONCE("Entering SingleAgentControl.........");
        try{
                listener_.waitForTransform(odom_frame_, base_frame_, ros::Time(0), ros::Duration(5.0));
                listener_.lookupTransform(odom_frame_, base_frame_,  ros::Time(0), transform_);
            }
        catch (tf::TransformException &ex) 
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                tf_error_ = true;
            }
        double delta_obs_x = 0.0;
        double delta_obs_y = 0.0;
        double delta_x = 0.0;
        double delta_y = 0.0;
        double delta_obstacle = 0.0;
         if(!tf_error_)
        {   
            pos_feedback_.x = transform_.getOrigin().x();
            pos_feedback_.y = transform_.getOrigin().y();
            pos_goal_.x = pos_start_.x + dis_goal;
            pos_goal_.y = pos_start_.y;
            pos_desired_.x = pos_start_.x;
            pos_desired_.y =  pos_start_.y;
            pos_obstacle_.x =  pos_start_.x +  obstacle_.x;
            pos_obstacle_.y =  pos_start_.y +  obstacle_.y;
            dis_delta_goal =  dis_goal;
//             ROS_INFO_STREAM("x_t = "<<  pos_feedback_.x);
//             ROS_INFO_STREAM("y_t = "<<  pos_feedback_.y);
            ROS_INFO_STREAM("obs_x_t = "<<  pos_obstacle_.x);
            ROS_INFO_STREAM("obs_y_t = "<<  pos_obstacle_.y);
            ROS_INFO_STREAM("pos_goal_.x = "<<  pos_goal_.x);
            ROS_INFO_STREAM("pos_goal_.y= "<<  pos_goal_.y);
            t_run_ = ros::Time::now();
            ROS_INFO_STREAM("====================================== ");
    //         ROS_INFO_STREAM("---agv.vel_x: " <<  vel_feedback_.x); 
    //         ROS_INFO_STREAM("---agv.vel_y: " <<  vel_feedback_.y);
            double_xy B;
            if( pos_feedback_.x <  pos_goal_.x &&  dis_delta_goal >  tolerance_dis)
            {
                 pos_desired_.x = ( pos_start_.x +  vel_desired_.x * (t_run_ - t_start_).toSec()) <  pos_goal_.x ? ( pos_start_.x +  vel_desired_.x * (t_run_ - t_start_).toSec()): pos_goal_.x;
                 pos_desired_.y =  pos_start_.y;
                ROS_INFO_STREAM("===========vel.x: " <<  vel_feedback_.x); 
                ROS_INFO_STREAM("===========vel.y: " <<  vel_feedback_.y);
                ROS_INFO_STREAM("===========vel_desired_.x: " <<  vel_desired_.x); 
                ROS_INFO_STREAM("===========vel_desired_.y: " <<  vel_desired_.y);
                ROS_INFO_STREAM("x_d = "<<  pos_desired_.x << "\ty_d = "<<  pos_desired_.y);
                ROS_INFO_STREAM("x_t = "<<  pos_feedback_.x << "\ty_t = "<<  pos_feedback_.y);
                dis_delta_goal = sqrt(pow(( pos_goal_.x -  pos_feedback_.x),2) + pow(( pos_goal_.y -  pos_feedback_.y),2));
                ROS_INFO_STREAM("dis_delta_goal = "<< dis_delta_goal);
                delta_obstacle =  sqrt(pow(( pos_obstacle_.x -  pos_feedback_.x),2) + pow(( pos_obstacle_.y -  pos_feedback_.y),2));
                ROS_INFO_STREAM("delta_obstacle = "<< delta_obstacle);
                

                
                B.x =  B_k * (  pos_feedback_.x -  pos_obstacle_.x );
                B.y =  B_k * (  pos_feedback_.y -  pos_obstacle_.y );
                
                delta_x =  pos_desired_.x- pos_feedback_.x;
                delta_y =  pos_desired_.y- pos_feedback_.y;
                ROS_INFO_STREAM("delta_x = "<< delta_x << "\tdelta_y = "<< delta_y);
                
                delta_obs_x = pos_obstacle_.x - pos_feedback_.x;
                delta_obs_y = pos_obstacle_.y - pos_feedback_.y;
                
                double F1_x_temp =  vel_desired_.x +  vel_pid_k_.x * delta_x -  lambda*B.x;
                double F1_y_temp =  vel_desired_.y +  vel_pid_k_.y * delta_y -  lambda*B.y;
                
                double F1_x,F1_y;
                    if (F1_x_temp > (-1)* vel_limit_.x && F1_x_temp <  vel_limit_.x)
                    {
                            F1_x = F1_x_temp;
                    }else if(F1_x_temp < (-1)* vel_limit_.x){
                        F1_x = (-1)* vel_limit_.x;
                    }else{
                        F1_x =  vel_limit_.x;
                    }
                    if (F1_y_temp > (-1)* vel_limit_.y && F1_y_temp <  vel_limit_.y){
                            F1_y = F1_y_temp;
                    }else if(F1_y_temp < (-1)* vel_limit_.y){
                        F1_y = (-1)* vel_limit_.y;
                    }else{
                        F1_y =  vel_limit_.y;
                    }
                    
                double Bright =  Bright_k * ((pow(( pos_feedback_.x -  pos_obstacle_.x),2) + pow(( pos_feedback_.y -  pos_obstacle_.y),2)) - pow( dis_safe,2));
                
                double F2 =( lambda+B.x* vel_feedback_.x+B.y *  vel_feedback_.y + Bright)<0.0 ? 0.0:( lambda + B.x *  vel_feedback_.x +B.y *  vel_feedback_.y+ Bright);
                
                
                //update lambda rate
                double lambda_rate = (F2 -  lambda) / ( epsilon_k *  epsilon);
                ROS_INFO_STREAM("lambda_rate = "<< lambda_rate);
                
                //update vel rate
                double_xy cmd_vel_rate;
                cmd_vel_rate.x = (F1_x -  vel_feedback_.x) /  epsilon; 
                cmd_vel_rate.y = (F1_y -  vel_feedback_.y) /  epsilon; 
                ROS_INFO_STREAM("cmd_vel_rate_x = "<< cmd_vel_rate.x << "\tcmd_vel_rate_y = "<< cmd_vel_rate.y);
                double cmd_vel_temp_x,cmd_vel_temp_y;
                cmd_vel_temp_x =  vel_feedback_.x + cmd_vel_rate.x * (t_run_ - t_last_).toSec();
                cmd_vel_temp_y =  vel_feedback_.y + cmd_vel_rate.y * (t_run_ - t_last_).toSec();
                    if (cmd_vel_temp_x > (-1)* vel_limit_.x  && cmd_vel_temp_x <  vel_limit_.x ){
                            move_cmd_.linear.x = cmd_vel_temp_x;
                    }else if(cmd_vel_temp_x < (-1)* vel_limit_.x ){
                        move_cmd_.linear.x = (-1)* vel_limit_.x ;
                    }else{
                        move_cmd_.linear.x =  vel_limit_.x ;
                    }
                    if (cmd_vel_temp_y > (-1)* vel_limit_.y && cmd_vel_temp_y <  vel_limit_.y){
                            move_cmd_.linear.y = cmd_vel_temp_y;
                    }else if(cmd_vel_temp_y < (-1)* vel_limit_.y){
                        move_cmd_.linear.y = (-1)* vel_limit_.y;
                    }else{
                        move_cmd_.linear.y =  vel_limit_.y;
                    }
                ROS_INFO_STREAM("v_x_t = "<<  vel_feedback_.x << "\tmove_cmd.linear.x = "<< move_cmd_.linear.x);
                ROS_INFO_STREAM("v_y_t = "<<  vel_feedback_.y << "\tmove_cmd.linear.y = "<< move_cmd_.linear.y);
                move_cmd_.linear.z = 0.0;
                move_cmd_.angular.x = 0.0;
                move_cmd_.angular.y = 0.0;
                move_cmd_.angular.z = 0.0;
                ROS_INFO_STREAM("delta_time = "<< (t_run_ - t_last_).toSec());
                 lambda =  lambda - lambda_rate * (t_last_ -t_run_).toSec();
                ROS_INFO_STREAM(" lambda = "<<  lambda);
                t_last_ = t_run_;
                cmd_pub_.publish(move_cmd_);
                delta_dis_pub_.publish(delta_dis_);
                delta_dis_.pose.position.x = delta_obs_x;
                delta_dis_.pose.position.y = delta_obs_y;
                delta_dis_.pose.position.z = delta_obstacle;
                delta_dis_.header.frame_id = base_frame_;
                delta_dis_.header.stamp = ros::Time::now();
                delta_dis_.pose.orientation.w = lambda;
                delta_dis_.pose.orientation.x = delta_x;
                delta_dis_.pose.orientation.y = delta_y;
                delta_dis_.pose.orientation.z = 0.0;
                
            }
            else{
                ROS_INFO_STREAM("stop.... ");
                delta_dis_.pose.position.x = delta_obs_x;
                delta_dis_.pose.position.y = delta_obs_y;
                delta_dis_.pose.position.z = delta_obstacle;
                delta_dis_.header.frame_id = base_frame_;
                delta_dis_.header.stamp = ros::Time::now();
                delta_dis_.pose.orientation.w = lambda;
                delta_dis_.pose.orientation.x = delta_x;
                delta_dis_.pose.orientation.y = delta_y;
                delta_dis_.pose.orientation.z = 0.0;
                move_cmd_.linear.x = 0.0;
                move_cmd_.linear.y = 0.0;
                move_cmd_.linear.z = 0.0;
                move_cmd_.angular.x = 0.0;
                move_cmd_.angular.y = 0.0;
                move_cmd_.angular.z = 0.0;
                cmd_pub_.publish(move_cmd_);
                delta_dis_pub_.publish(delta_dis_);
            }
        }
        
}


void Multibot::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
//      ROS_INFO_STREAM_ONCE("Entering odom_callback.........");
     odom_resv_ = *odom_msg;
      vel_feedback_.x = odom_resv_.twist.twist.linear.x;
      vel_feedback_.y = odom_resv_.twist.twist.linear.y;
     
//      ROS_INFO_STREAM_ONCE("odom_frame_recv: " << odom_resv_.header.frame_id);
//      ROS_INFO_STREAM("===========odomcallback_vel.x: " <<  vel_feedback_.x); 
//      ROS_INFO_STREAM("===========odomcallback_vel.y: " <<  vel_feedback_.y);
}
