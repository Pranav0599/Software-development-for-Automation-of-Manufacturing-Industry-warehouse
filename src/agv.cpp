#include "agv.h"

AGV::AGV(ros::NodeHandle &node_handler, int id)
{
    nh = node_handler;    
    agv_id = std::to_string(id);
    agv_name = "agv"+ std::to_string(id);

    init();
}

void AGV::init()
{   
    ROS_INFO_STREAM(""<< agv_name<< " is initialized" );
    agv_station_sub = nh.subscribe("/ariac/"+agv_name+"/station", 10, &AGV::set_agv_station, this);
    agv_state_sub = nh.subscribe("/ariac/"+agv_name+"/state", 10, &AGV::set_agv_state, this);
    agv_order_sub = nh.subscribe("/ariac/orders", 10, &AGV::set_agv_destination, this);
    quality_sensor_sub = nh.subscribe("/ariac/quality_control_sensor_"+agv_id, 10, &AGV::check_faulty_part, this);
}

void AGV::set_agv_state(const std_msgs::String::ConstPtr &state) 
{
    if(state->data == "ready_to_deliver" && !shipment_delivered)
    {
        ROS_INFO("Shipment ready to submit");
        send_agv_to_as();
        shipment_delivered = true;
    }
}

void AGV::set_agv_station(const std_msgs::String::ConstPtr &station) 
{
    agv_station = station->data;
    ROS_INFO_STREAM("Agv station set to:" << station->data);
}

void AGV::set_agv_destination(const nist_gear::Order::ConstPtr &orders)
{
    order= *orders;

    if(sizeof(order.kitting_shipments[0])/sizeof(order.kitting_shipments[0]) != 0)
    {

        if(order.kitting_shipments[0].agv_id == agv_name)
        {
            agv_destination = order.assembly_shipments[0].station_id;
            agv_shipment_type = order.assembly_shipments[0].shipment_type;
        }
    }
}

void AGV::check_faulty_part(const nist_gear::LogicalCameraImage::ConstPtr &quality_sensor_msg)
{   
    if(agv_station == "ks"+agv_id)
    {
        if(quality_sensor_msg->models.size() > 0)
        {
            ROS_INFO_STREAM_ONCE("Faulty Part detected in "<< agv_name);
        }
    }   
}

void AGV::send_agv_to_as()
{
    ros::ServiceClient shipment_submit_client = nh.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/"+agv_name+"/submit_shipment");
    if (!shipment_submit_client.exists())
    {
        ROS_INFO("Waiting for the agv submit shipment service to start...");
        shipment_submit_client.waitForExistence();
    }

    ROS_INFO("Requesting the service...");
    nist_gear::AGVToAssemblyStation srv_msg;
    // ROS_INFO(srv_msg);
    srv_msg.request.assembly_station_name = agv_destination;
    srv_msg.request.shipment_type = agv_shipment_type;

    shipment_submit_client.call(srv_msg);

    if (!srv_msg.response.success)
    {
        ROS_ERROR_STREAM("Failed to submit agv to assembly station " << srv_msg.response.message);
    }
    else
    {
        ROS_INFO("AGV moving to:");
    }


}

