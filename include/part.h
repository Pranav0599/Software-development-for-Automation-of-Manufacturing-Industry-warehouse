#include <iostream>
#include <geometry_msgs/Pose.h>
#include <string.h>

/**
 * @brief The part class gives the details of the part and sets or gets the faulty parts.
 * 
 */

class Part{
    private:
        std::string part_type;
        std::string part_color;
        bool is_faulty;
        geometry_msgs::Pose part_pose;
        ros::Time creation_time;

    public:
        /**
         * @brief Get the part type 
         * 
         * @return std::string 
         */
        std::string get_part_type();

        /**
         * @brief Set the part type 
         * 
         */
        void set_part_type(std::string type);

        /**
         * @brief Get the part color 
         * 
         * @return std::string 
         */
        std::string get_part_color();

        /**
         * @brief Set the part color object
         * 
         */
        void set_part_color(std::string color);


        /**
         * @brief Get the part is faulty
         * 
         * @return true 
         * @return false 
         */
        bool get_is_faulty();

        /**
         * @brief Set the part as faulty 
         * 
         */
        void set_is_faulty(bool status);

        /**
         * @brief Get the part pose 
         * 
         * @return geometry_msgs::Pose 
         */
        geometry_msgs::Pose get_part_pose();

        /**
         * @brief Set the part pose
         * 
         */
        void set_part_pose(geometry_msgs::Pose &pose);

        /**
         * @brief Get the creation time of part
         * 
         * @return ros::Time
         */
        ros::Time get_creation_time();

        /**
         * @brief Set the creation time of part
         * 
         */
        void set_creation_time(ros::Time time);
       
        
};