#include <ros/ros.h>
#include "visualizer.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizer_node");
    ros::NodeHandle nh("~");

    Visualizer visualizer(nh);
    
    ros::spin();

    return 0;
}
