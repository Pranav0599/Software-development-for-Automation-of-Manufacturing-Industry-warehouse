#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <nist_gear/AGVToAssemblyStation.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Model.h>
#include <group2_rwa2/check_exists.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>


/**
 * @brief The AGV class comtains the details of part and respective assembly station coordinates
 * 
 */

class BinsHandler{

    private:
    ros::NodeHandle nh;

    ros::Subscriber logicam_bin0_sub;
    ros::Subscriber logicam_bin1_sub;  
    ros::ServiceServer check_part_service_bins0; 
    ros::ServiceServer check_part_service_bins1; 

    int bins0_parts_count; 
    std::vector<nist_gear::Model> bins0_parts;
    int bins1_parts_count; 
    std::vector<nist_gear::Model> bins1_parts;

    public:    

    /**
     * @brief Construct a new Bins Handler object
     * 
     * @param node_handler 
     */


    BinsHandler(ros::NodeHandle &node_handler);

    void init();

    /**
     * @brief Callback function for bin 1
     * 
     * @param logicam_msg 
     */

    void logicam_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

    /**
     * @brief Callback function for bin 1
     * 
     * @param logicam_msg 
     */

    void logicam_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

    /**
     * @brief Callback function to check requested part service for bin0
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */

    bool check_part_service_bins0_callback(group2_rwa2::check_exists::Request &req, group2_rwa2::check_exists::Response &res);

    /**
     * @brief Callback function to check requested part service for bin1
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */

    bool check_part_service_bins1_callback(group2_rwa2::check_exists::Request &req, group2_rwa2::check_exists::Response &res);

};