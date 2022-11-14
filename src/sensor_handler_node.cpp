#include "sensor_handler_node.h"



void SensorHandler::quality_control_sensor_1_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{

    if(msg->models.size() > 0)
    {
        ROS_INFO_STREAM("Faulty part detected on AGV1:" << msg->models.size());
    }      

};

void SensorHandler::quality_control_sensor_2_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{

    if(msg->models.size() > 0)
    {
        ROS_INFO_STREAM("Faulty part detected on AGV2:" << msg->models.size());
    }      

};


void SensorHandler::quality_control_sensor_3_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{

    if(msg->models.size() > 0)
    {
        ROS_INFO_STREAM("Faulty part detected on AGV3:" << msg->models.size());
    }
    
};

void SensorHandler::quality_control_sensor_4_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{

    if(msg->models.size() > 0)
    {
        ROS_INFO_STREAM("Faulty part detected on AGV4:" << msg->models.size());
    }      

};



int main(int argc, char ** argv){

    ros::init(argc, argv, "ariac_sensor_handler");
    // usleep(20000000);

    ros::NodeHandle nh;
    SensorHandler sensor_handler(nh);

    ros::Subscriber breakbeam_sensor_sub = nh.subscribe("/ariac/break_beam_0_change", 10, &SensorHandler::breakbeam_sensor_callback, &sensor_handler);
    ros::Subscriber proximity_sensor_sub= nh.subscribe("/ariac/proximity_sensor_0", 10, &SensorHandler::proximity_sensor_callback, &sensor_handler);
    ros::Subscriber logical_camera_subscriber = nh.subscribe("/ariac/logical_camera_bins0", 10, &SensorHandler::logical_camera_callback, &sensor_handler);
    ros::Subscriber quality_control_subsriber_1 = nh.subscribe("/ariac/quality_control_sensor_1", 10, &SensorHandler::quality_control_sensor_1_callback, &sensor_handler);
    ros::Subscriber quality_control_subsriber_2 = nh.subscribe("/ariac/quality_control_sensor_2", 10, &SensorHandler::quality_control_sensor_2_callback, &sensor_handler);
    ros::Subscriber quality_control_subsriber_3 = nh.subscribe("/ariac/quality_control_sensor_3", 10, &SensorHandler::quality_control_sensor_3_callback, &sensor_handler);
    ros::Subscriber quality_control_subsriber_4 = nh.subscribe("/ariac/quality_control_sensor_4", 10, &SensorHandler::quality_control_sensor_4_callback, &sensor_handler);

    ros::spin(); 

    return 0;

}