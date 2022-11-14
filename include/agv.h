#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <nist_gear/Order.h>
#include <nist_gear/AGVToAssemblyStation.h>
#include <nist_gear/LogicalCameraImage.h>


/**
 * @brief The AGV class comtains the details of part and respective assembly station coordinates
 * 
 */

class AGV{

    private:
    ros::NodeHandle nh;
    std::string agv_id;
    std::string agv_name;
    std::string agv_station;
    std::string agv_state;
    std::string agv_destination;
    std::string agv_shipment_type;
    bool shipment_delivered;

    nist_gear::Order order;
    ros::Subscriber agv_station_sub;
    ros::Subscriber agv_state_sub;
    ros::Subscriber agv_order_sub;
    ros::Subscriber quality_sensor_sub;

    /**
     * @brief Defining the required functions for the AGV
     * 
     */

    public:
    

    AGV(ros::NodeHandle &node_handler, int id);

    /**
     * @brief Initializing 
     * 
     */

    void init();

    /**
     * @brief Set the agv state 
     * 
     * @param state 
     */
    
    void set_agv_state(const std_msgs::String::ConstPtr &state);

    /**
     * @brief Set the agv station
     * 
     * @param station 
     */

    void set_agv_station(const std_msgs::String::ConstPtr &station);

    /**
     * @brief Set the agv destination 
     * 
     * @param orders 
     */

    void set_agv_destination(const nist_gear::Order::ConstPtr & orders);

    /**
     * @brief Check if part is faulty
     * 
     * @param quality_sensor_msg 
     */

    void check_faulty_part(const nist_gear::LogicalCameraImage::ConstPtr &quality_sensor_msg);

    /**
     * @brief Send AGV to assembly station
     * 
     */

    void send_agv_to_as();

};
