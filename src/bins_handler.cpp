#include "bins_handler.h"

BinsHandler::BinsHandler(ros::NodeHandle &node_handler)
{
    nh = node_handler;    
    init();
}

void BinsHandler::init()
{   
    logicam_bin0_sub = nh.subscribe("/ariac/logical_camera_bins0", 10, &BinsHandler::logicam_bins0_callback, this);
    logicam_bin1_sub = nh.subscribe("/ariac/logical_camera_bins1", 10, &BinsHandler::logicam_bins1_callback, this);
    check_part_service_bins0 = nh.advertiseService("/group2/check_part_bins0", &BinsHandler::check_part_service_bins0_callback, this);
    check_part_service_bins1 = nh.advertiseService("/group2/check_part_bins1", &BinsHandler::check_part_service_bins1_callback, this);
}

void BinsHandler::logicam_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        bins0_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            bins0_parts.push_back(logicam_msg->models[i]);
        }
        bins0_parts_count = bins0_parts.size();
        ROS_INFO_STREAM_ONCE("Bin 0 part count : " << bins0_parts_count);
    }
}

void BinsHandler::logicam_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        bins1_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            bins1_parts.push_back(logicam_msg->models[i]);
        }
        bins1_parts_count = bins1_parts.size();
        ROS_INFO_STREAM_ONCE("Bin 1 part count : " << bins1_parts_count);
    }
}

bool BinsHandler::check_part_service_bins0_callback(group2_rwa2::check_exists::Request &req, group2_rwa2::check_exists::Response &res)
{       
    for(int i = 0; i < bins0_parts_count; i++)
    {        
        if(bins0_parts[i].type == req.part_type)
        {
            ROS_INFO_STREAM(req.part_type);
            res.success = true;
        }
    }    
    return true;
}

bool BinsHandler::check_part_service_bins1_callback(group2_rwa2::check_exists::Request &req, group2_rwa2::check_exists::Response &res)
{       
    for(int i = 0; i < bins1_parts_count; i++)
    {        
        if(bins1_parts[i].type == req.part_type)
        {
            ROS_INFO_STREAM(req.part_type);
            res.success = true;
        }
    }    
    return true;
}