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

class SensorHandler
{
    public:
    ros::NodeHandle nh;

    SensorHandler(ros::NodeHandle & node_handler)
    {
        nh = node_handler;
    }


    /**
     * @brief Callback function for brekbeam sensor
     * 
     * @param sensor_msg 
     */
    void breakbeam_sensor_callback(const nist_gear::Proximity::ConstPtr &sensor_msg) 
    {
        if (sensor_msg->object_detected) 
        {  
            ROS_INFO("Break beam triggered.");
        }
    }

    /**
     * @brief Callback function for logical camera sensor
     * 
     * @param camera_msgs 
     */
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr &camera_msgs)
    {
        ROS_INFO_STREAM_THROTTLE(10,"Logical camera: '" << camera_msgs->models.size() << "' objects.");
    }

    /**
     * @brief Callback function for Proximity sensor
     * 
     * @param msg 
     */
    void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg)
    {
        if ((msg->max_range - msg->range) > 0.01)
        {
            ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
        }
    }

    /**
     * @brief Callback function for quality control sensor 1
     * 
     * @param msg 
     */

    void quality_control_sensor_1_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);


    /**
     * @brief Callback function for quality control sensor 2
     * 
     * @param msg 
     */
    void quality_control_sensor_2_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

    /**
     * @brief Callback function for quality control sensor 3
     * 
     * @param msg 
     */
    void quality_control_sensor_3_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
    
    /**
     * @brief Callback function for quality control sensor 4
     * 
     * @param msg 
     */
    void quality_control_sensor_4_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
};