#ifndef PROCESS_IMAGE_H_
#define PROCESS_IMAGE_H_


#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "ball_chaser/DriveToTarget.h"


class process_image
{
    public:
    process_image(ros::NodeHandle* nodehandle);
    

    private:
    ros::NodeHandle* nh_;
    ros::Subscriber sub1;
    ros::ServiceClient client;
    float lin_x_ {0.0};
    float ang_z_ {0.0};

    void fake_func();
    

    void init_subscriber();
    void init_serviceclient();
    void drive_robot(float lin_x, float ang_z);
    void process_image_callback(const sensor_msgs::Image img);


    
        

};

process_image::process_image(ros::NodeHandle* nodehandle) :nh_(nodehandle)
{
    init_subscriber();
    init_serviceclient();
}


void process_image::init_subscriber()
{
    ROS_INFO("Sub started");
    sub1 = nh_->subscribe("/camera/rgb/image_raw", 10, &process_image::process_image_callback,this);

}

void process_image::init_serviceclient()
{
    ROS_INFO("Service started");
    client = nh_->serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
}



void process_image::drive_robot(float lin_x, float ang_z)
{
   
    ball_chaser::DriveToTarget srv; 
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z_;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service command_robot");
    }
        
    
}



void process_image::process_image_callback(const sensor_msgs::Image img)
{

    //ROS_INFO("Image callback Height %lu Width %lu",(unsigned int)img.height,(unsigned int)img.width);
    //width and height is 800 by 800
     int white_pixel = 255;

     bool ball_found= false;
     int row =0;                //number of row
     int step_length = img.step; //length of row
     int step=0;                //row step increments
     int i=0;  
     int step_found =0;

    //Parse of entire img.data 
     for(row =0;row<img.height; row++ )
     {

         for(step =0; step<= img.step; step++)
         {
             //ROS_INFO("Parsing");
             i = row*step_length+step;
             if(img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] ==white_pixel)
             {
                 //ROS_INFO("Ball found");
                 step_found =step;
                 ball_found = true;
                 break;
                
             }
         }

     }

     int onethird_img = step/3;  //width /3
     
    
     if(ball_found == true)
     {
         ROS_INFO("step %d", step_found);
         //ROS_INFO("col %d", step);
         //ROS_INFO("Width %d", onethird_img);
         if(step_found<onethird_img)
         {
            ROS_INFO("Left");
            lin_x_ = 0.1;
            ang_z_ =0.1;
            drive_robot(lin_x_,ang_z_);
         }
        if(step_found <= (onethird_img*2) && step_found > (onethird_img))
         {
            ROS_INFO("Middle");
            lin_x_ = 0.2;
            ang_z_ =0;
            drive_robot(lin_x_,ang_z_);
         }
        if(step_found <= (onethird_img * 3 ) && step_found > (onethird_img*2) ) 
        {
            //Turn right
            ROS_INFO("Right");
            lin_x_ = 0.1;
            ang_z_ =-0.1;
            drive_robot(lin_x_,ang_z_);
        }
        
     }
     else
     {
         //stop
         ROS_INFO("Stop");
         lin_x_ = 0;
         ang_z_ =0;
         drive_robot(lin_x_,ang_z_);
     }
    

}






#endif