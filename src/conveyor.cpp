#include "../include/conveyor.h"

Conveyor::Conveyor(ros::NodeHandle &nh)
{
    sensor_sub = nh.subscribe("/ariac/proximity_sensor_0", 10, &Conveyor::sensor_sub_callback, this);
    init();
};

void Conveyor::init()
{
    ROS_INFO("Conveyor Handler Node Started");
};

void Conveyor::add_part_to_conveyor(Part part)
{
    conveyor_parts.push_back(part);    
    conveyor_part_count = conveyor_parts.size();
};

void Conveyor::dispose_part()
{
    //method under construction
};

Part Conveyor::get_conveyor_part(std::string part_name)
{
    for(int i = 0; i<conveyor_parts.size(); i++)
    {
        if (conveyor_parts.at(i).get_part_type() == part_name)
        {
            ROS_INFO("Part retrieved successfully");
            return conveyor_parts.at(i);
        }
        else
        {
            ROS_ERROR("Part does not exist !!");
        }
    }
};


int Conveyor::get_remaining_part_count()
{

    return conveyor_part_count;
};

void Conveyor::sensor_sub_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if ((msg->max_range - msg->range) > 0.01)
    {
        Part part;
        geometry_msgs::Pose pose;

        pose.position.y = msg->range;
        part.set_part_pose(pose);
        part.set_creation_time(ros::Time::now());
        add_part_to_conveyor(part);

        ROS_INFO_STREAM("New part detected - Part Pose: [" << pose.position.x << ", " << pose.position.y << ", "<< pose.position.z << " ]");
        ros::Duration(3.0).sleep(); //sensor will sleep for 3 seconds
    }
};