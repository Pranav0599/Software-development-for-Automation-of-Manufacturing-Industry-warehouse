#include "../include/main_handler.h"


MainHandler::MainHandler(ros::NodeHandle & node_handler)
{
    nh = node_handler;
    init();
}

void MainHandler::init()
{
    ROS_INFO("Main handler started");
    competion_state_sub = nh.subscribe("/ariac/competition_state", 10, &MainHandler::competition_state_callback, this);
}

void MainHandler::competition_state_callback(const std_msgs::String::ConstPtr& msg)
{   
    if(msg->data == "init")
    {
         start_competition();
    };
}

bool MainHandler::start_competition()
{   
    competition_start_client = MainHandler::nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        
    if (!competition_start_client.exists())
    {
        competition_start_client.waitForExistence();
    };

    ROS_INFO("Requesting competition start....");
    std_srvs::Trigger srv; 
    competition_start_client.call(srv); 

    if (!srv.response.success)
    {
        ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
        return false;
    }
    else
    {
        ROS_INFO("Competition started!");
        return true;
    }
}
