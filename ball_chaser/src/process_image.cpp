#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "process_image.h"


int main(int argc, char ** argv)
{

    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
    process_image image(&n);
    ros::spin();
    return 0;
}