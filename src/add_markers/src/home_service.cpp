#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>

// Define a global client that can request services
ros::Publisher marker_pub;
int state = 0;
ros::Time timer;

bool close_enough(float x1, float y1, float x2,float y2, float d)
{
    double dist = pow( pow(x1 - x2, 2) + pow(y1 - y2, 2) , 0.5 );
    ROS_INFO("distance - %f", dist);
    if (dist < d) return true;
    else return false;
}
void wait_until_its_time()
{
   if (ros::Time::now() - timer >  ros::Duration(5.0)) {
      state++;
   }
} 
void create_marker(double x, double y, bool remove)
{
    
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    if (remove) marker.action = visualization_msgs::Marker::DELETEALL;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.9;

    marker.lifetime = ros::Duration();
    // Publish the marker
    marker_pub.publish(marker);
    state++;
}

void odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_INFO("poses    %f  %f",msg->pose.pose.position.x, msg->pose.pose.position.y );
  double goal_x = -1.0;
  const double goal_y = -1.0;
  switch (state)
  {
    case 0: // no marker
      create_marker(goal_x, goal_y, false);
      break;
    case 1: // first marker exists
      if (close_enough(goal_x, goal_y,  msg->pose.pose.position.x, msg->pose.pose.position.y, 0.1)){
         create_marker(goal_x, goal_y, true);
         timer = ros::Time::now();
      }
      break;
    case 2: // first marker reached - waiting
      wait_until_its_time();
      break;
    case 3: // no marker
      goal_x = 4.0;
      create_marker(goal_x, goal_y, false);
      break;
    case 4: // second marker up
      goal_x = 4.0;
      if (close_enough(goal_x, goal_y,  msg->pose.pose.position.x, msg->pose.pose.position.y, 0.1)) create_marker(goal_x, goal_y, true);
      break;

  } 
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "home_service"); 
  ros::NodeHandle n;
  
  // Define a client service capable of requesting services from add_markers
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
  // Subscribe to odom topic n
  ros::Subscriber sub1 = n.subscribe("amcl_pose", 10, odom_callback);

  // Handle ROS communication events
  ros::spin();
  return 1;
}

