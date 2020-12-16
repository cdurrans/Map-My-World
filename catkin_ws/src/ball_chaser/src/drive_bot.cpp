#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
                          ball_chaser::DriveToTarget::Response& res)
{

    float linear_x = (float)req.linear_x;
    float angular_z = (float)req.angular_z;

    ROS_INFO("DriveToTarget received - lx:%1.2f, az:%1.2f", linear_x, angular_z);

    geometry_msgs::Twist motor_command;
    motor_command.linear.x = linear_x;
    motor_command.angular.z = angular_z;
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Current velocities: linear_x -- " + std::to_string(linear_x) + " angular_z -- " + std::to_string(angular_z);
    ROS_INFO_STREAM(res.msg_feedback);
    
    ros::Duration(0.01).sleep();

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Command Robot is ready");
    
    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}