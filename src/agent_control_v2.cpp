#include "multi_agent_control/agent_control.h"

int main(int argc,char** argv)
{
    ROS_INFO("Multi robot system is initializing...");
    ros::init(argc,argv,"agent_control");
    Multibot multirobot;
    multirobot.loop();
    return 0;
}
