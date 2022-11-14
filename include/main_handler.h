#include <vector>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_srvs/Trigger.h>

class MainHandler
{   
    private:
    ros::NodeHandle nh;
    ros::Subscriber competion_state_sub;
    ros::ServiceClient competition_start_client;

    public:
    
    MainHandler(ros::NodeHandle & node_handler);

    void init();


    /**
     * @brief getting the compition state
     * 
     * @param msg 
     */
    void competition_state_callback(const std_msgs::String::ConstPtr& msg);



    /**
     * @brief To start the compition if the state exisits, if not it will return failed msg
     * 
     * @return true 
     * @return false 
     */
    bool start_competition();
};