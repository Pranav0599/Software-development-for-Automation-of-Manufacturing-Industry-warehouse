#include <vector>
#include <string>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nist_gear/Order.h>
#include <nist_gear/Model.h>


class OrderHandler{

    private:    
    ros::Subscriber orders_sub;
    ros::NodeHandle nh;

    std::vector<nist_gear::Order> incomplete_orders;
    std::vector<nist_gear::Order> completed_orders;

    bool high_priority_order_recieved;

    std::string current_order_id;
    std::string order_kitting_shipment_type;
    std::string order_kitting_agv_id;
    std::string order_kitting_destination_id;
    std::vector<nist_gear::Model> order_kitting_products;

    std::string order_assembly_station_id;
    std::string order_assembly_shipment_type;
    std::vector<nist_gear::Model> order_shipment_products;

    public:
    OrderHandler(ros::NodeHandle &nodehandler);
    void init();

    /**
     * @brief Get the recent order
     * 
     * @return nist_gear::Order 
     */
    nist_gear::Order get_recent_order();

     /**
     * @brief Get the completed order count
     * 
     * @return int 
     */
    int get_completed_order_count();

    /**
     * @brief Get the incomplete order count
     * 
     * @return int 
     */
    int get_incomplete_order_count();

    /**
     * @brief Func to process the order recieved
     * 
     * @param order 
     */
    void process_orders(const nist_gear::Order::ConstPtr &order);

};