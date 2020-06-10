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
/*
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>*/


class Multibot
{
    public:
        Multibot();
        ~Multibot();
        void loop();
    
    private:
        bool initMultibot();
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

            nav_msgs::Odometry Odom_resv_;
            double v_x_t;
            double v_y_t;
            
            std::string odom_frame_;


};

#endif /* INCLUDE_AGENT_CONTROL_H_ */
