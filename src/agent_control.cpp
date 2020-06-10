#include "multi_agent_control/agent_control.h"

Multibot::Multibot(){}
Multibot::~Multibot(){}

void Multibot::loop()
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_("~");

    nh_p_.param<std::string>("odom_frame",odom_frame_,std::string("odom"));
    nh_p_.param<std::string>("base_frame",base_frame_,std::string("base_footprint"));
    
    if(initMultibot())
    
    
}


bool Multibot::initMultibot()
{
    
}
