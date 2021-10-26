#ifndef DRIVE_BOT_H_
#define DRIVE_BOT_H_


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class drive_bot
{
    public:
    drive_bot(ros::NodeHandle* nodehandle);
    

    private:
    ros::NodeHandle* nh_;
    ros::Publisher motor_command_publisher_;
    geometry_msgs::Twist motor_command_;
    ros::ServiceServer service_;
    bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &resp);
    void init_motor_command_pub();
    void init_motor_service();
    


    

};

drive_bot::drive_bot(ros::NodeHandle* nodehandle) :nh_(nodehandle)
{

    
    //init publisher and srv
    init_motor_command_pub();
    init_motor_service();
}


void drive_bot::init_motor_command_pub()
{
    motor_command_publisher_= nh_->advertise<geometry_msgs::Twist>("/cmd_vel",10);
}



void drive_bot::init_motor_service()
{
    ROS_INFO("Service started");
    ros::ServiceServer service_ = nh_->advertiseService("/ball_chaser/command_robot",&drive_bot::handle_drive_request,this);
}

bool drive_bot::handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
{
    ROS_INFO("DriveToTargetRequest received - j1:%1.2f, j2:%1.2f", (float)req.linear_x, (float)req.angular_z);

    motor_command_.linear.x = req.linear_x;
    motor_command_.angular.z = req.angular_z;

    motor_command_publisher_ .publish(motor_command_);

    res.msg_feedback = "Motor command set - linear_x: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);

    //a message feedback should be returned with the requested wheel velocities

    return true;


}


#endif