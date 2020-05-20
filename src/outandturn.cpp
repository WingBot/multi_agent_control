#include "ros/ros.h"
#include "std_msgs/String.h"//geometry_msgs
#include "geometry_msgs/Twist.h"//包含elocity space消息
#include <tf/transform_listener.h>
#include "math.h"
#include <sstream>
#include <iostream>
// #include <rbx1_nav/CalibrateAngularConfig.h>
// #include <rbx1_nav/CalibrateLinearConfig.h>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"out_and_back");//指定节点“out_and_back”
    ros::NodeHandle n;//创造一个节点句柄
    ros::Publisher cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);//将在/cmd_vel话题上发布一个geometry_msgs::Twist消息
    int rate=20;//定义更新频率
    ros::Rate loop_rate(rate);//更新频率20Hz，它会追踪记录自上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间
 
    //初始化操作
    double linear_speed=0.05;//向前的线速度0.2m/s
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
      listener.lookupTransform("/odom", "base_footprint",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
 
    double x_start = transform.getOrigin().x();
    double y_start = transform.getOrigin().y();
    double angle_start = acos(transform.getRotation().z())*2;
    double distance = 0.0;
    double angle = 0.0;
    cout<<"angle_start: "<<angle_start<<endl;
    while(ros::ok())//等待键盘ctrl+C操作则停止
    {
        tf::StampedTransform transform_;
        try{
          listener.waitForTransform("/odom", "base_footprint", ros::Time(0), ros::Duration(1.0));
          listener.lookupTransform("/odom", "base_footprint",
                                   ros::Time(0), transform_);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        move_cmd.linear.x = linear_speed;
        if(distance < goal_distance)
        {
            cmd_vel_pub.publish(move_cmd);
            loop_rate.sleep();
            distance = sqrt(pow((transform_.getOrigin().x() - x_start),2)+pow((transform_.getOrigin().y() - y_start),2));
            cout<<"distance: "<<distance<<endl;
            cout<<"odom.x: "<<transform_.getOrigin().x()<<endl;
            cout<<"odom.y: "<<transform_.getOrigin().y()<<endl;
        }
        else {
            if(angle + angular_tolerance < goal_angle){
                move_cmd.linear.x=0.0;
                move_cmd.angular.z=angular_speed;
                cmd_vel_pub.publish(move_cmd);
                loop_rate.sleep();
                angle = abs(acos(transform_.getRotation().z())*2 - angle_start);
                cout<<"angle: "<<angle<<endl;
            }
            else{
                move_cmd.linear.x=0.0;
                move_cmd.angular.z=0.0;
                cmd_vel_pub.publish(move_cmd);
                loop_rate.sleep();
                cout<<"stop!"<<endl;
            }
            }
    }
return 0;
}
