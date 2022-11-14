#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <cstdlib>
#include <nist_gear/AssemblyStationSubmitShipment.h>
#include <vector>
#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include <std_srvs/Trigger.h>
#include <unistd.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <nist_gear/AGVToAssemblyStation.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>


/**
 * @brief Class for handling Assembly operation
 * 
 */
class AssemblyHandler
{
    private:
    std::string assembly_station_id;
    std::string assembly_station_name;
    std::string shipment_type;
    std::string agv_name;
    std::string assembly_shipment_type;
    bool assembly_delivered;
    nist_gear::Order order; 
    
    ros::NodeHandle nh;
    ros::Subscriber as_order_sub;

    public:

    /**
     * @brief Construct a new Assembly Handler node handler
     * 
     * @param node_handler 
     */

    AssemblyHandler(ros::NodeHandle &node_handler, int id);    

    void init();


    /**
     * @brief assembly_station_submit_shipment func will submit the shipment at assembly station
     * 
     */
    void assembly_station_submit_shipment();

    /**
     * @brief Func to process the order
     * 
     * @param orders 
     */

    void process_order(const nist_gear::Order::ConstPtr & orders);
    
};