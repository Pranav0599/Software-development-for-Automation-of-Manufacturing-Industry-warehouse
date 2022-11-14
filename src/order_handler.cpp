#include "order_handler.h"


OrderHandler::OrderHandler(ros::NodeHandle &nodehandler)
{
    nh = nodehandler;

    init();
}

void OrderHandler::init()
{
    orders_sub = nh.subscribe("/ariac/orders", 10, &OrderHandler::process_orders, this);
}

void OrderHandler::process_orders(const nist_gear::Order::ConstPtr &order)
{   
    incomplete_orders.push_back(*order);
    ROS_INFO("New order recieved");   
    
}

int OrderHandler::get_completed_order_count()
{
    return completed_orders.size();
}

int OrderHandler::get_incomplete_order_count()
{
    return incomplete_orders.size();
}
