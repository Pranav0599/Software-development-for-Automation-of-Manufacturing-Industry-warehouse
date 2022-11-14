#include <ros/ros.h>
#include "../include/conveyor.h"

int main(int argc, char ** argv)
{   
    ros::init(argc, argv, "conveyor_handler_node");

    ros::NodeHandle nh;
    Conveyor conveyor_handler(nh);

    ros::spin();

}