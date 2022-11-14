#include "../include/part.h"




std::string Part::get_part_type()
{
    return part_type;
};

void Part::set_part_type(std::string type)
{
    part_type = type;
};

std::string Part::get_part_color()
{
    return part_color;
};

void Part::set_part_color(std::string color)
{
    part_color = color;
};


bool Part::get_is_faulty()
{
    return is_faulty;
};

void Part::set_is_faulty(bool status)
{
    is_faulty = status;
};

geometry_msgs::Pose Part::get_part_pose()
{
    return part_pose;
};

void Part::set_part_pose(geometry_msgs::Pose & pose)
{
    part_pose = pose;
};

ros::Time Part::get_creation_time()
{
    return creation_time;
};

void Part::set_creation_time(ros::Time time)
{
    creation_time = time;
};