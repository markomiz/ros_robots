#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)) ROS_ERROR("Failed to call drive bot service");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int white_position = -1;

     // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height * img.step; i+=3) {
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
            white_position = i;
            break;
        }
    }
    
    // Then, identify if this pixel falls in the left, mid, or right side of the image
	int distance_from_left = white_position % img.step;
	ROS_INFO("width %i", img.step);
	ROS_INFO("left dist %i", distance_from_left);

	// Depending on the white ball position, call the drive_bot function and pass velocities to it

	std::vector<float> velocities = {0.5, 0}; // unless identifies as being in the left or right below, robot goes straight

	if (distance_from_left < img.step / 3) // right third
	{
		velocities = {0, 0.5};
	}
	else if (distance_from_left > 2 * img.step / 3) // left third
	{
		velocities = {0, -0.5};
	}

    
    // Request a stop when there's no white ball seen by the camera
	if (white_position == -1)
	{
		velocities = {0, 0};
	}
	
	drive_robot(velocities[0], velocities[1]);

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
