#include "b_agility_challenge_flip.h"

Flip::Flip(ros::NodeHandle &node_handler)
{
    nh = node_handler;
    init();
}

void Flip::init()
{
    part_flip_sub = nh.subscribe("/ariac/orders", 10, &Flip::order_callback, this);
}

void Flip::order_callback(const nist_gear::Order::ConstPtr &order)
{
    for (int i=0; i< order->kitting_shipments[0].products.size(); i++)
    {
        poses.push_back(order->kitting_shipments[0].products[i].pose);
    }     
    ROS_INFO("Order received");
    compare();
    
}

void Flip::compare()
{
    for(int i =0; i <poses.size(); i++)
    {
        float roll = poses[i].orientation.x;
        float quat = poses[i].orientation.w;
        if (roll == 1 && quat!=1)
        {
            ROS_INFO_STREAM("Order Part "<<i<<" needs to be flipped");
        }
    }    
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "flipped_part");
    ROS_INFO("Flip detector Initialized");

    ros::NodeHandle nh;
    Flip order(nh);   

    ros::spin();
}