#include "assembly_handler.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"assembly_handler_node");
    
    ros::NodeHandle nh;

    AssemblyHandler as1(nh, 1);
    AssemblyHandler as2(nh, 2);
    AssemblyHandler as3(nh, 3);
    AssemblyHandler as4(nh, 4);

    ros::spin();

}