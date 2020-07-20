#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void publish_marker(const char* ros_message) {
  
  // Publish the marker
  ROS_INFO(ros_message);
  marker_pub.publish(marker);
  
  ROS_INFO("Sleeping for 5 seconds");
  sleep(5);
}

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  
  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  
  // Publish the marker at the pick up zone
  marker.pose.position.x = 1.0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.action = visualization_msgs::Marker::ADD;
  publish_marker("Initialize pick up object");
  
  // Make the marker invisible
  marker.action = visualization_msgs::Marker::DELETE;
  publish_marker("Object being picked up");
  
  // Publish the marker at the drop off zone
  marker.pose.position.x = -2.0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  
  marker.action = visualization_msgs::Marker::ADD;
  publish_marker("Object delivered to drop off zone");
  
  return 0;
}