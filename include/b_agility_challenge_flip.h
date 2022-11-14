#include <ros/ros.h>
#include <nist_gear/Order.h>

#include "part.h"

/**
 * @brief Flip class to handle Fliped part agility chanllange
 * 
 */

class Flip
{
    private:
    ros::NodeHandle nh;
    ros::Subscriber part_flip_sub;    
    std::vector<geometry_msgs::Pose> poses;
    

    public:

    /**
     * @brief Construct a new Flip object
     * 
     * @param node_handler 
     */

    Flip(ros::NodeHandle &node_handler);

    void init();

    /**
     * @brief Callback function to recieve order
     * 
     * @param order 
     */

    void order_callback(const nist_gear::Order::ConstPtr &order);

    /**
     * @brief Function to compare the position values
     * 
     */

    void compare();
   

    
    
};