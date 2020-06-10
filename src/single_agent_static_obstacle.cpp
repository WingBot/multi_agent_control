#include "ros/ros.h"
#include "std_msgs/String.h"//geometry_msgs
#include "geometry_msgs/Twist.h"//包含elocity space消息
#include <tf/transform_listener.h>
#include "math.h"
#include <sstream>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

// #include <rbx1_nav/CalibrateAngularConfig.h>
// #include <rbx1_nav/CalibrateLinearConfig.h>

//                /|\ x
//                 |
//                 |
//  y              |
// <---------------
using namespace std;
nav_msgs::Odometry Odom_resv_;
double v_x_t=0.0;//向前的线速度0.2m/s
double v_y_t=0.0;//向前的线速度0.2m/s
boost::mutex odom_mutex_;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& Odom_msg)
{
//     Odom_resv_ = *Odom_msg;
//     v_x_t = Odom_resv_.twist.twist.linear.x;
//     v_y_t = Odom_resv_.twist.twist.linear.y;
//     ROS_INFO_STREAM("OdomCallback_________v_x_t = "<< v_x_t);
//     ROS_INFO_STREAM("OdomCallback_________v_y_t = "<< v_y_t);
    ROS_INFO_STREAM("OdomCallback_________ = ");
//         try
//     {
//         odom_mutex_.lock();
// //         last_twist_time_ = ros::Time::now();
//         Odom_resv_ = *Odom_msg.get();
//         ROS_INFO_STREAM("OdomCallback_________ = "<< Odom_resv_.twist.twist.linear.x);
//         odom_mutex_.unlock();
//     }
//     catch(...)
//     {
//         odom_mutex_.unlock();
//     }
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"static_obstacle_avoidance");//指定节点“out_and_back”
    
    ros::NodeHandle n;//创造一个节点句柄
    
    ros::Publisher cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);//将在/cmd_vel话题上发布一个geometry_msgs::Twist消息
    ros::Publisher delta_dis_pub=n.advertise<geometry_msgs::PoseStamped>("/delta_dis",1000);
    
//     ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry> ("/odom", 5, OdomCallback);
    int parameter;
    bool ifget1 = ros::param::get("param1", parameter);
    int rate=20;//定义更新频率
    bool tf_error = false;
    ros::Rate loop_rate(rate);//更新频率20Hz，它会追踪记录自上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间
 
    
    
    
    //初始化操作
    double v_x_d=0.2;//向前的线速度0.2m/s
    double v_y_d=0.0;//向前的线速度0.2m/s
    double v_x_max=0.6;//向前的线速度0.2m/s
    double v_y_max=0.6;//向前的线速度0.2m/s

    double v_x_k = 2.0;
    double v_y_k = 2.0;
    double obstacle_x_set = 1.0;
    double obstacle_y_set = 0.1;
    

    double safe_distance = 0.2;//初始化障碍物位置与安全距离
    double delta_obstacle = 0.0;
    double delta_dis_x = 0.0;
    double delta_dis_y = 0.0;
    double goal_distance=2.0;//行进记录1.0m
    double angular_speed=0.1;//角度素1.0rad/s
    double goal_angle=M_PI;
    double angular_tolerance = 0.5*M_PI/180;//角度容忍度
    tf::TransformListener listener;
   
    geometry_msgs::Twist move_cmd;//定义消息对象
    geometry_msgs::PoseStamped delta_dis;//
    move_cmd.linear.x=move_cmd.linear.y=move_cmd.linear.z=0;
    move_cmd.angular.x=move_cmd.angular.y=move_cmd.angular.z=0;
    delta_dis.pose.position.x = delta_dis.pose.position.y = delta_dis.pose.position.z = 0.0;
    tf::StampedTransform transform;
    try{
            listener.waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("odom", "base_footprint",  ros::Time(0), transform);
        }
    catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            tf_error = true;
        }
    
//     double x_start = 0.0;
//     double y_start = 0.0;
    
    double x_start = transform.getOrigin().x();
    double y_start = transform.getOrigin().y();
    double x_t = transform.getOrigin().x();
    double y_t = transform.getOrigin().y();
    double x_g = x_start+goal_distance;
    double y_g = y_start;
    double x_d = x_start;
    double y_d = y_start;
    double obstacle_x = x_start+obstacle_x_set;
    double obstacle_y = y_start+obstacle_y_set;


    ros::Time t_start = ros::Time::now();
    double goal_error = 0.01;//=========================到达目标容忍度0.01m
    
    
    double delta_goal = goal_distance;
    
    //double angle_start = acos(transform.getRotation().z())*2;
    double angle_start = 0.0;
    double distance = 0.0;
    double angle = 0.0;

    double B_x = 0.0;
    double B_y = 0.0;
    double Bright = 0.0;

    
    double epsilon = 0.1;//set to 0.1
    double lambda_x = 0.0;
//     double lambda_y = 0.0;
    double Bright_k = -50.0;
    double B_k = -2 ;
    
    double delta_x = 0.0;
    double delta_y = 0.0;
    
    ros::Time t_run;
    ros::Time t_last;
    
//     cout<<"angle_start: "<<angle_start<<endl;
    ROS_INFO_STREAM_ONCE("x_start = "<< transform.getOrigin().x());
    ROS_INFO_STREAM_ONCE("y_start = "<< transform.getOrigin().x());
    ROS_INFO_STREAM_ONCE("angle_start = "<< transform.getOrigin().x());
    while(ros::ok())//等待键盘ctrl+C操作则停止
    {
        tf::StampedTransform transform_;
        try
            {
                listener.waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(5.0));
                listener.lookupTransform("odom", "base_footprint", ros::Time(0), transform_);
            }
        catch (tf::TransformException &ex) 
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                tf_error = true;
            }
        
        if(!tf_error)
        {   
            ROS_INFO_STREAM("\n");
            ROS_INFO_STREAM("====================================== ");
            //获取当前位置
            x_t = transform_.getOrigin().x();
            y_t = transform_.getOrigin().y();
            t_run = ros::Time::now();
//             x_d = v_x_d * (t_run - t_start).toSec();
            //计算期望位置
//             x_d = (x_start + v_x_d * (t_run - t_start).toSec()) < x_g ? (x_start +v_x_d * (t_run - t_start).toSec()):x_g;
//             y_d = y_start;

//             ROS_INFO_STREAM_ONCE("need to followe or not ? ");
            //判断与目标位置距离是否到达终点，避免航向偏移，加判断直线位移是否到达
           if(x_t < x_g && delta_goal > goal_error)
            {
                x_d = (x_start + v_x_d * (t_run - t_start).toSec()) < x_g ? (x_start +v_x_d * (t_run - t_start).toSec()):x_g;
                y_d = y_start;
                ROS_INFO_STREAM("x_d = "<< x_d << "\ty_d = "<< y_d);
                ROS_INFO_STREAM("x_t = "<< x_t << "\ty_t = "<< y_t);
                cmd_vel_pub.publish(move_cmd);
                delta_dis_pub.publish(delta_dis);
                loop_rate.sleep();
//                 distance = sqrt(pow((transform_.getOrigin().x() - x_start),2)+pow((transform_.getOrigin().y() - y_start),2));
                //计算与目标位置的距离
                delta_goal = sqrt(pow((x_g - x_t),2) + pow((y_g - y_t),2));
                ROS_INFO_STREAM("delta_goal = "<< delta_goal);
                //计算与障碍物的距离,topic
                delta_obstacle =  sqrt(pow((obstacle_x - x_t),2) + pow((obstacle_y - y_t),2));
                ROS_INFO_STREAM("delta_obstacle = "<< delta_obstacle);
                //计算过程函数F1、F2
                

                
                //忽略太小的偏差
//                 delta_x = abs(x_d - x_t) < 0.0000000000000000000001 ? 0.0 : (x_d - x_t); 
//                 delta_y = abs(y_d - y_t) < 0.0000000000000000000001 ? 0.0 : (y_d - y_t); //0的范围限定
                delta_dis_x = obstacle_x - x_t;
                delta_dis_y = obstacle_y - y_t;
                
                delta_x = x_d - x_t; 
                delta_y = y_d - y_t; //0的范围限定
                //topic
                ROS_INFO_STREAM("delta_x = "<< delta_x << "\tdelta_y = "<< delta_y);
//                 v_x_t = Odom_resv_.twist.twist.linear.x;
//                 v_y_t = Odom_resv_.twist.twist.linear.y;
                v_x_t = v_x_d;
                v_y_t = v_y_d;
//                 v_x_t = abs(v_x_d + v_x_k * delta_x) > v_x_max ? v_x_max : (v_x_d + v_x_k * delta_x);
//                 v_y_t = abs(v_y_d + v_y_k * delta_y) > v_x_max ? v_x_max : (v_y_d + v_y_k * delta_y);
                
                B_x = B_k * ( x_t - obstacle_x );
                B_y = B_k * ( y_t - obstacle_y );
                ROS_INFO_STREAM("( x_t - obstacle_x ) = "<< ( x_t - obstacle_x ) << "\t( y_t - obstacle_y ) = "<< ( y_t - obstacle_y ));
                ROS_INFO_STREAM("B_x = "<< B_x << "\tB_y = "<< B_y);
                
                double F1_x_temp = v_x_d + v_x_k * delta_x - lambda_x * B_x;
                double F1_y_temp = v_y_d + v_y_k * delta_y - lambda_x * B_y;
                double F1_x,F1_y;
                    if (F1_x_temp > (-1)*v_x_max && F1_x_temp < v_x_max){
                            F1_x = F1_x_temp;
                    }else if(F1_x_temp < (-1)*v_x_max){
                        F1_x = (-1)*v_x_max;
                    }else{
                        F1_x = v_x_max;
                    }
                    if (F1_y_temp > (-1)*v_y_max && F1_y_temp < v_y_max){
                            F1_y = F1_y_temp;
                    }else if(F1_y_temp < (-1)*v_y_max){
                        F1_y = (-1)*v_y_max;
                    }else{
                        F1_y = v_y_max;
                    }
//                 double F1_x = abs(v_x_d + v_x_k * delta_x - lambda_x * B_x) > v_x_max ? v_x_max : (v_x_d + v_x_k * delta_x - lambda_x * B_x);
//                 double F1_y = abs(v_y_d + v_y_k * delta_y - lambda_y * B_y) > v_y_max ? v_y_max : (v_y_d + v_y_k * delta_y - lambda_y * B_y);
//                 ROS_INFO_STREAM();
                
                Bright = Bright_k * ((pow((x_t - obstacle_x),2) + pow((y_t - obstacle_y),2)) - pow(safe_distance,2));
                double F2_x = (lambda_x + B_x * v_x_t + B_y * v_y_t + Bright) < 0.0 ? 0.0 : (lambda_x + B_x * v_x_t +B_y * v_y_t+ Bright);
//                 double F2_y = (lambda_y   + Bright) < 0.0 ? 0.0 : (lambda_y + B_y * v_y_t + Bright);
                ROS_INFO_STREAM("F1_x Vel= "<< F1_x << "\tF1_y Vel= "<< F1_y<<"\tF2_x = "<< F2_x);
                
                
                //参数更新率
                double lambda_rate_x = (F2_x - lambda_x) / (2 * epsilon);
//                 double lambda_rate_y = (F2_y - lambda_y) / (10 * epsilon);
                ROS_INFO_STREAM("lambda_rate_x = "<< lambda_rate_x);
                
                //速度指令更新率
                double cmd_vel_rate_x = (F1_x - v_x_t) / epsilon; 
                double cmd_vel_rate_y = (F1_y - v_y_t) / epsilon; 
                ROS_INFO_STREAM("cmd_vel_rate_x = "<< cmd_vel_rate_x << "\tcmd_vel_rate_y = "<< cmd_vel_rate_y);
                //速度指令赋值更新与参数更新
                
                
//                 v_x_t = v_x_t + cmd_vel_rate_x * (t_run - t_last).toSec();
//                 v_y_t = v_y_t + cmd_vel_rate_y * (t_run - t_last).toSec();
                double cmd_vel_temp_x,cmd_vel_temp_y;
                cmd_vel_temp_x = v_x_t + cmd_vel_rate_x * (t_run - t_last).toSec();
                cmd_vel_temp_y = v_y_t + cmd_vel_rate_y * (t_run - t_last).toSec();
                    if (cmd_vel_temp_x > (-1)*v_x_max && cmd_vel_temp_x < v_x_max){
                            move_cmd.linear.x = cmd_vel_temp_x;
                    }else if(cmd_vel_temp_x < (-1)*v_x_max){
                        move_cmd.linear.x = (-1)*v_x_max;
                    }else{
                        move_cmd.linear.x = v_x_max;
                    }
                    if (cmd_vel_temp_y > (-1)*v_y_max && cmd_vel_temp_y < v_y_max){
                            move_cmd.linear.y = cmd_vel_temp_y;
                    }else if(cmd_vel_temp_y < (-1)*v_y_max){
                        move_cmd.linear.y = (-1)*v_y_max;
                    }else{
                        move_cmd.linear.y = v_y_max;
                    }
                
                
//                 move_cmd.linear.x = v_x_t + cmd_vel_rate_x * (t_run - t_last).toSec();
//                 move_cmd.linear.y = v_y_t + cmd_vel_rate_y * (t_run - t_last).toSec();
                
                ROS_INFO_STREAM("v_x_t = "<< v_x_t << "\tmove_cmd.linear.x = "<< move_cmd.linear.x);
                ROS_INFO_STREAM("v_y_t = "<< v_y_t << "\tmove_cmd.linear.y = "<< move_cmd.linear.y);
                
                
                move_cmd.linear.z = 0.0;
                move_cmd.angular.x = 0.0;
                move_cmd.angular.y = 0.0;
                move_cmd.angular.z = 0.0;
                
                
                ROS_INFO_STREAM("delta_time = "<< (t_run - t_last).toSec());
                ROS_INFO_STREAM("move_cmd.linear.x = "<< move_cmd.linear.x<< "\tmove_cmd.linear.y = "<< move_cmd.linear.y);
//                 ROS_INFO_STREAM();
//                 lambda_x = lambda_x;
                lambda_x = lambda_x - lambda_rate_x * (t_last -t_run).toSec();
//                 lambda_y = lambda_y + lambda_rate_y * (t_last -t_run).toSec();
                ROS_INFO_STREAM("lambda_xs = "<< lambda_x);
//                 ROS_INFO_STREAM();
                t_last = t_run;
                delta_dis.pose.position.x = delta_dis_x;
                delta_dis.pose.position.y = delta_dis_y;
                delta_dis.pose.position.z = delta_obstacle;
                delta_dis.header.frame_id = "base_footprint";
                delta_dis.header.stamp = ros::Time::now();
                delta_dis.pose.orientation.w = lambda_x;
                delta_dis.pose.orientation.x = delta_x;
                delta_dis.pose.orientation.y = delta_y;
                delta_dis.pose.orientation.z = 0.0;
                
//              ROS_INFO_STREAM("y_d = "<< y_d);

//                 ROS_INFO_STREAM("delta_y = "<< delta_y);
                
            }
            else {
                ROS_INFO_STREAM_ONCE("stop.... ");
                delta_dis.pose.position.x = delta_dis_x;
                delta_dis.pose.position.y = delta_dis_y;
                delta_dis.pose.position.z = delta_obstacle;
                delta_dis.header.frame_id = "base_footprint";
                delta_dis.header.stamp = ros::Time::now();
                delta_dis.pose.orientation.w = lambda_x;
                delta_dis.pose.orientation.x = delta_x;
                delta_dis.pose.orientation.y = delta_y;
                delta_dis.pose.orientation.z = 0.0;
                move_cmd.linear.x = 0.0;
                move_cmd.linear.y = 0.0;
                move_cmd.linear.z = 0.0;
                move_cmd.angular.x = 0.0;
                move_cmd.angular.y = 0.0;
                move_cmd.angular.z = 0.0;
                cmd_vel_pub.publish(move_cmd);
                delta_dis_pub.publish(delta_dis);
//                 if(angle + angular_tolerance < goal_angle ){
//                     move_cmd.linear.x=0.0;
//                     move_cmd.angular.z=angular_speed;
//                     cmd_vel_pub.publish(move_cmd);
//                     loop_rate.sleep();
//                     angle = abs(acos(transform_.getRotation().z())*2 - angle_start);
//                     cout<<"angle: "<<angle<<endl;
//                 }
//                 else{
//                     move_cmd.linear.x=0.0;
//                     move_cmd.angular.z=0.0;
//                     cmd_vel_pub.publish(move_cmd);
//                     loop_rate.sleep();
//                     cout<<"stop!"<<endl;
//                 }
                }
        }
    }
return 0;
}
