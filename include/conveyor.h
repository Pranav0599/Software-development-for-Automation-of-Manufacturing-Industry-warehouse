#include <iostream>
#include <vector>
#include "../include/part.h"
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

/**
 * @brief The Conveyor class contains the information of parts remaining on it and what parts 
          are spawned. The part details are inherited from part class.
 * 
 */

class Conveyor{
    private:
        std::vector<Part> conveyor_parts;
        int conveyor_part_count;
        ros::Subscriber sensor_sub;
       

    public:
        /**
         * @brief Construct a new Conveyor object
         * 
         * @param nh 
         */
        explicit Conveyor(ros::NodeHandle &nh);
        /**
         * @brief initializing the conveyor class
         * 
         */
        void init();

        /**
         * @brief Get the part object
         * 
         * 
         * @return part
         */
        Part get_conveyor_part(std::string part_name);

        /**
         * @brief Adding parts to store 
         * 
         * @param part 
         */
        void add_part_to_conveyor(Part part);
        

        /**
         * @brief Get the remaining part count 
         * 
         * @return int 
         */
        int get_remaining_part_count();

        /**
         * @brief callback method to get sensor msgs from proximity sensor
         * 
         * @param msg 
         */
        void sensor_sub_callback(const sensor_msgs::Range::ConstPtr &msg);

        /**
         * @brief part that falls off of the conveyor
         * 
         */
        void dispose_part();


};