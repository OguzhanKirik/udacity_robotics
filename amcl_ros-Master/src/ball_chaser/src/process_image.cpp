#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

#include <vector>
// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if(client.call(srv))
        ROS_INFO("Successfull service call");
    else
        ROS_INFO("Failed service call");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    std::vector<int> white_pixels;
    int left_border = img.width / 3;
    int right_border = (img.width / 3)*2;
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    for (size_t i = 0; i < (img.height * img.step); i=i+3){
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel 
        && img.data[i+2] == white_pixel){
                white_pixels.emplace_back((i/3 ) % img.width);
        }
        
    }
    if (!white_pixels.empty()){

    
        // find the section
        int middle_pixel = white_pixels.at(white_pixels.size()/2);
        

        if(middle_pixel <= left_border){
            drive_robot(0.1,0.2);

            ROS_INFO_STREAM("White Ball on left, approaching!!!");

        }else if(middle_pixel > left_border && middle_pixel <=right_border ){
            drive_robot(0.5,0.0);
            ROS_INFO_STREAM("White Ball in middle, approaching!!!");

        }else {
            drive_robot(0.1,-0.2);
            ROS_INFO_STREAM("White Ball on right, approaching!!!");

        }
    }else{
        ROS_INFO_STREAM("No White Ball, stopping!!!");
        drive_robot(0.0, 0.0);

    }


}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
