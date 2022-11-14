#include "insufficient_parts_challenge.h"

InsufficientParts::InsufficientParts(ros::NodeHandle &node_handler)
{
    nh = node_handler;
    init();
}

void InsufficientParts::init()
{
    order_sub = nh.subscribe("/ariac/orders", 10, &InsufficientParts::process_orders, this);
    logicam_conveyor_sub = nh.subscribe("/ariac/logical_camera_conveyor", 10, &InsufficientParts::get_conveyor_part, this);
    logicam_bins0_sub = nh.subscribe("/ariac/logical_camera_bins0", 10, &InsufficientParts::get_bins0_parts, this);
    logicam_bins1_sub = nh.subscribe("/ariac/logical_camera_bins1", 10, &InsufficientParts::get_bins1_parts, this);   

    clock_sub = nh.subscribe("/clock", 10, &InsufficientParts::check_parts, this);    
    
}

void InsufficientParts::process_orders(const nist_gear::Order::ConstPtr &orders)
{
    // ROS_INFO("Getting the Parts from Orders...");
    orders_part_type_list.clear();
    if(orders->kitting_shipments[0].products.size() > 0)
    {
        for (int i=0; i< orders->kitting_shipments[0].products.size(); i++)
        {
            orders_part_type_list.push_back(orders->kitting_shipments[0].products[i].type);
        }
    }
    // ROS_INFO("Stored the Parts from Order!!");
}

void InsufficientParts::get_conveyor_part(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    // ROS_INFO("Getting the Parts from Conveyor...");
    conveyor_parts_list.clear();
    if(logicam_msg->models.size() > 0)
    {
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            conveyor_parts_list.push_back(logicam_msg->models[i].type);
        }
    }

    // ROS_INFO("Stored the Parts from Conveyor!!");

    
}

void InsufficientParts::get_bins0_parts(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{ 
    // ROS_INFO("Getting the Parts from Bin 0...");
    bins0_parts_list.clear();
    if(logicam_msg->models.size() > 0)
    {
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            bins0_parts_list.push_back(logicam_msg->models[i].type);
        }
    }

    // ROS_INFO("Stored the Parts from Bin 0!!");
}

void InsufficientParts::get_bins1_parts(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{ 
    // ROS_INFO("Getting the Parts from Bin 1...");
    bins1_parts_list.clear();
    if(logicam_msg->models.size() > 0)
    {
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            bins1_parts_list.push_back(logicam_msg->models[i].type);
        }
    }
}


void InsufficientParts::check_parts(const rosgraph_msgs::Clock &time_sec)
{   
    
    if((time_sec.clock.sec % 5) == 0 && orders_part_type_list.size() > 0)
    {
        ROS_INFO_STREAM("Checking for Sufficient Parts at " << time_sec.clock.sec<< " second");

        bool parts_found = false;
        // checking in the bins 0
        for(int i=0; i< orders_part_type_list.size(); i++)
        {   
            bool is_found = false;
            for(int j=0; j< bins0_parts_list.size(); j++)
            {
                if( orders_part_type_list[i] == bins0_parts_list[j])
                {
                    is_found = true;  
                }
            }

            if(is_found == false)
            {
                unfound_parts.push_back(orders_part_type_list[i]);
                is_found = false;
            }
        }

        // checking in the bins 0
        if(unfound_parts.size() != 0)
        {        
            for(int i=0; i< unfound_parts.size(); i++)
            {   
                bool is_found = false;
                for(int j=0; j< bins1_parts_list.size(); j++)
                {
                    if( unfound_parts[i] == bins1_parts_list[j])
                    {
                        is_found = true;
                    }
                }

                if(is_found == true)
                {
                    unfound_parts.erase(unfound_parts.begin()+i);
                }
            }
        }

        // checking in the conveyor parts
        // ros::Duration(10.0).sleep();
        if(unfound_parts.size() != 0)
        {   
            for(int i=0; i< unfound_parts.size(); i++)
            {   
                bool is_found = false;
                for(int j=0; j< conveyor_parts_list.size(); j++)
                {
                    if( unfound_parts[i] == conveyor_parts_list[j])
                    {
                        is_found = true;
                    }
                }

                if(is_found == true)
                {
                    unfound_parts.erase(unfound_parts.begin()+i);
                }
            }
        }
        

        if(unfound_parts.size() == 0)
        {
            ROS_INFO("Sufficient Parts exist");
        }
        else
        {
            ROS_INFO_STREAM("Insufficient Parts count " << unfound_parts.size());
        }
        unfound_parts.clear();
        ros::Duration(2.0).sleep();
    }
}

int main(int argc, char ** argv){

    ros::init(argc, argv, "insufficient_parts_challenge");
    // usleep(20000000);
    ROS_INFO("Insufficient Parts Challenge initialized");

    ros::NodeHandle nh;
    InsufficientParts insufficient_parts_checker(nh); 

    ros::spin(); 
    return 0;

}