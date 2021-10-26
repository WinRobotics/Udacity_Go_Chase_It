#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "drive_bot.h"


int main(int argc, char ** argv)
{

    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;
    drive_bot bot(&n);
    ros::spin();
    return 0;
}