#include <vector>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_srvs/Trigger.h>
#include <unistd.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nist_gear/DetectedProduct.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <rosgraph_msgs/Clock.h>

/**
 * @brief Class to handle the Insufficient Parts Challenge
 * 
 */
class InsufficientParts
{
    private:
        ros::NodeHandle nh;

        std::vector<std::string> orders_part_type_list;
        std::vector<std::string> bins0_parts_list;
        std::vector<std::string> bins1_parts_list;
        std::vector<std::string> conveyor_parts_list;
        std::vector<std::string> unfound_parts;

        ros::Subscriber order_sub;
        ros::Subscriber logicam_bins0_sub;
        ros::Subscriber logicam_bins1_sub;
        ros::Subscriber logicam_conveyor_sub;
        ros::Subscriber clock_sub;

    public:
        InsufficientParts(ros::NodeHandle &node_handler); 
        
        /**
         * @brief Subscribe to the orders Topic
         * 
         */
        void init();

        /**
         * @brief Get all the part_type from the Order
         * 
         * @param sensor_handler 
         * @return std::vector<std::string> 
         */
        void process_orders(const nist_gear::Order::ConstPtr &orders);

        /**
         * @brief Get all the part_type from the Logical Camera over the Bin
         * 
         * @param logicam_msg 
         */
        void get_bins0_parts(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

        /**
         * @brief Get all the part_type from the Logical Camera over the Bin
         * 
         * @param logicam_msg 
         */

        void get_bins1_parts(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

        /**
         * @brief Get all the part_type from the Logical Camera over the Conveyor
         * 
         * @param logicam_msg 
         */
        void get_conveyor_part(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

        /**
         * @brief Check is sufficient parts exist.
         * 
         */
        void check_parts(const rosgraph_msgs::Clock &time_sec);
};

