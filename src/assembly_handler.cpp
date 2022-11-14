#include "assembly_handler.h"


AssemblyHandler::AssemblyHandler(ros::NodeHandle &node_handler, int id)
{
    nh = node_handler;
    assembly_station_id = std::to_string(id);
    assembly_station_name = "as"+assembly_station_id;

    init();
}

void AssemblyHandler::init()
{
    as_order_sub = nh.subscribe("/ariac/orders", 10, &AssemblyHandler::process_order, this);
}
/**
 * @brief assembly_station_submit_shipment func will submit the assembly station
 * 
 */
void AssemblyHandler::assembly_station_submit_shipment()
{
    
    ros::ServiceClient client_as = nh.serviceClient<nist_gear::AssemblyStationSubmitShipment>("/ariac/"+assembly_station_name+"/submit_shipment");
    
    if (!client_as.exists())
    {
        ROS_INFO("Submitting Assembly Station 1");
        client_as.waitForExistence();
    }

    ROS_INFO("Requesting the service...");
    nist_gear::AssemblyStationSubmitShipment srv_msg;
    srv_msg.request.shipment_type = assembly_shipment_type;
    client_as.call(srv_msg);
    ROS_INFO_STREAM("Submitted assembly station " << assembly_station_name);
}

void AssemblyHandler::process_order(const nist_gear::Order::ConstPtr & orders)
{
    order= *orders;

    if(sizeof(order.assembly_shipments[0])/sizeof(order.assembly_shipments[0]) != 0)
    {

        if(order.assembly_shipments[0].station_id == assembly_station_name)
        {
            agv_name = order.kitting_shipments[0].agv_id;
            assembly_shipment_type = order.assembly_shipments[0].shipment_type;
            assembly_station_submit_shipment();
            usleep(3000);

        }
    }
}
    
