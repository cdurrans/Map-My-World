#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget direction;
    direction.request.linear_x = lin_x;
    direction.request.angular_z = ang_z;
    
    if (!client.call(direction))
        ROS_ERROR("Failed to call service drive_robot");
}

int * determine_position(int current_position, int height, int step) {
    int current_height;
    static int position[2];
    for (int i = 0; i < height; i++) {
        if ((i * step) > current_position){
            current_height = i - 1;
            break;
        }
    }
    int current_step = (current_position - (current_height * step));
    if (current_step > step) {current_step--;}
    position[0] = current_height;
    position[1] = current_step;
    ROS_INFO("White found at -- height:%i, step:%i", current_height, current_step);
    return position;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    bool no_white = true;
    int *position_ptr;

    for (int i = 0; i < img.height * img.step; i+=3) {
        if (img.data[i] == white_pixel && img.data[i++] == white_pixel && img.data[i+2] == white_pixel) {
            position_ptr = determine_position(i,img.height,img.step);
            no_white = false;
            break;
        }
    }
    if (no_white == true) {
        ROS_INFO("White not found");
        drive_robot(0.0, 0.0); //stop
        return;
    }
    int position[] = {*position_ptr, *(position_ptr+1)};
    double left_max = 0.3334;
    double right_min = 0.6668;
    // ROS_INFO("Position - height:%i, step:%i", position[0], position[1]);
    // ROS_INFO("Step Size - :%i", img.step);
    
    double percent_step = (position[1] / (img.step * 1.0f));
    // ROS_INFO("Current Percent Step - %1.5f", percent_step);
    
    if (percent_step < left_max) {
        drive_robot(0.25, 0.5); //drive left
        return;
    }
    if (percent_step > left_max && percent_step < right_min) {
        drive_robot(0.5, 0.0); //drive straight
        return;
    }
    if (percent_step > right_min) {
        drive_robot(0.25, -0.5); //drive right
        return;
    }
    else {
        drive_robot(0.0, 0.0); //stop
        return;
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