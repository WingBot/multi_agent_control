#include "ros/ros.h"
#include "std_msgs/String.h"//geometry_msgs
#include "geometry_msgs/Twist.h"//包含elocity space消息
#include <tf/transform_listener.h>
#include "math.h"
#include <sstream>
#include <iostream>

// #include <rbx1_nav/CalibrateAngularConfig.h>
// #include <rbx1_nav/CalibrateLinearConfig.h>

//                /|\ x
//                 |
//                 |
//  y              |
// <---------------
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"follower_line");//指定节点“out_and_back”
    
    ros::NodeHandle n;//创造一个节点句柄
    
    ros::Publisher cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);//将在/cmd_vel话题上发布一个geometry_msgs::Twist消息
    int parameter;
    bool ifget1 = ros::param::get("param1", parameter);
    int rate=20;//定义更新频率
    bool tf_error = false;
    ros::Rate loop_rate(rate);//更新频率20Hz，它会追踪记录自上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间
 
    //初始化操作
    double v_x_d=0.1;//向前的线速度0.2m/s
    double v_y_d=0.0;//向前的线速度0.2m/s
    double v_x_t=0.0;//向前的线速度0.2m/s
    double v_y_t=0.0;//向前的线速度0.2m/s
    double v_k = 2.0;
    double goal_distance=1.0;//行进记录1.0m
    double angular_speed=0.1;//角度素1.0rad/s
    double goal_angle=M_PI;
    double angular_tolerance = 0.5*M_PI/180;//角度容忍度
    tf::TransformListener listener;
   
    geometry_msgs::Twist move_cmd;//定义消息对象
    move_cmd.linear.x=move_cmd.linear.y=move_cmd.linear.z=0;
    move_cmd.angular.x=move_cmd.angular.y=move_cmd.angular.z=0;
 
    tf::StampedTransform transform;
    try{
            listener.waitForTransform("/odom", "base_footprint", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/odom", "base_footprint",  ros::Time(0), transform);
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


    ros::Time t_start = ros::Time::now();
    double goal_error = 0.01;//=========================到达目标精度0.01m
    
    
    double delta_goal = goal_distance;
    //double angle_start = acos(transform.getRotation().z())*2;
    double angle_start = 0.0;
    double distance = 0.0;
    double angle = 0.0;
    
    ros::Time t_run;
//     cout<<"angle_start: "<<angle_start<<endl;
    ROS_INFO_STREAM_ONCE("x_start = "<< transform.getOrigin().x());
    ROS_INFO_STREAM_ONCE("y_start = "<< transform.getOrigin().y());
    ROS_INFO_STREAM_ONCE("angle_start = "<< acos(transform.getRotation().z())*2);
    while(ros::ok())//等待键盘ctrl+C操作则停止
    {
        tf::StampedTransform transform_;
        try
            {
                listener.waitForTransform("/odom", "base_footprint", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("/odom", "base_footprint", ros::Time(0), transform_);
            }
        catch (tf::TransformException &ex) 
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                tf_error = true;
            }
        
        if(!tf_error)
        {          
            x_t = transform_.getOrigin().x();
            y_t = transform_.getOrigin().y();
            t_run = ros::Time::now();
//             x_d = v_x_d * (t_run - t_start).toSec();
            x_d = (x_start + v_x_d * (t_run - t_start).toSec()) < x_g ? (x_start +v_x_d * (t_run - t_start).toSec()):x_g;
            y_d = y_start;
//             ROS_INFO_STREAM("x_t : "<< x_t);
//             ROS_INFO_STREAM("y_t : "<< y_t);
//             ROS_INFO_STREAM("x_d : "<< x_d);
//             ROS_INFO_STREAM("y_d : "<< y_d);
//             ROS_INFO_STREAM("v_x_t : "<< v_x_t);
//             ROS_INFO_STREAM("v_y_t : "<< v_y_t);
//             
//             ROS_INFO_STREAM("delta_goal : "<< delta_goal);
            ROS_INFO_STREAM_ONCE("need to followe or not ? ");
           if(x_t < x_g && delta_goal > goal_error)
            {
                ROS_INFO_STREAM("===============following================== ");
                cmd_vel_pub.publish(move_cmd);
                loop_rate.sleep();
//                 distance = sqrt(pow((transform_.getOrigin().x() - x_start),2)+pow((transform_.getOrigin().y() - y_start),2));
                delta_goal = sqrt(pow((x_g - x_t),2) + pow((y_g - y_t),2));
                v_x_t = v_x_d;
                double delta_y = 0.0;
//                 var = (y < 10) ? 30 : 40;
                delta_y = abs(y_d - y_t)<0.00000000001 ? 0.0 : (y_d - y_t); //0的范围限定
                
                v_y_t = v_y_d + v_k * delta_y;
                
                move_cmd.linear.x = v_x_t;
                move_cmd.linear.y = v_y_t;
                //cout<<"distance: "<<distance<<endl;
//                 cout<<"odom.x: "<<transform_.getOrigin().x()<<endl;
//                 cout<<"odom.y: "<<transform_.getOrigin().y()<<endl;
//                 ROS_INFO_STREAM("      ");
                
//                 ROS_INFO_STREAM("y_t = "<< y_t);
                ROS_INFO_STREAM("x_d = "<< x_d << "\ty_d = "<< y_d);
                ROS_INFO_STREAM("x_t = "<< x_t << "\ty_t = "<< y_t);
//              ROS_INFO_STREAM("y_d = "<< y_d);
                ROS_INFO_STREAM("move_cmd.linear.x = "<< move_cmd.linear.x);
                ROS_INFO_STREAM("move_cmd.linear.y = "<< move_cmd.linear.y);
                ROS_INFO_STREAM("delta_y = "<< delta_y);
                ROS_INFO_STREAM("delta_goal = "<< delta_goal);
            }
            else {
                ROS_INFO_STREAM_ONCE("stop.... ");
                move_cmd.linear.x = 0.0;
                move_cmd.linear.y = 0.0;
                cmd_vel_pub.publish(move_cmd);
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
