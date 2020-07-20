#include <utility>
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

const float EPSILON = 0.2f;

// Pickup and drop-off point
// Using pair of float for easiness
const std::pair<float, float> pick_up_point(1.0f, 0.0f);
const std::pair<float, float> drop_off_point(-2.0f, 0.0f);

enum RobotState {
  INITIALIZE,
  PICKING_UP,
  DROPPING_OFF,
  DONE
};

visualization_msgs::Marker marker;
ros::Publisher marker_pub;
RobotState robot_state;

bool has_reached_point(const geometry_msgs::Pose& robot_pose, std::pair<float, float> point) {
  return (fabs(robot_pose.position.x - point.first) < EPSILON)
    && (fabs(robot_pose.position.y - point.second) < EPSILON);
}

void publish_marker(const char* ros_message) {
  
  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  
  ROS_INFO(ros_message);
  marker_pub.publish(marker);
}

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  
  geometry_msgs::Pose robot_pose = msg->pose.pose;
  
  switch (robot_state) {
    case INITIALIZE: {
      // Publish the marker at the pick up zone
      marker.pose.position.x = pick_up_point.first;
      marker.pose.position.y = pick_up_point.second;
      marker.pose.position.z = 0;
      marker.action = visualization_msgs::Marker::ADD;
      publish_marker("Initialize pick up object");
      
      robot_state = PICKING_UP;
      break;
    }
    case PICKING_UP: {
      if (has_reached_point(robot_pose, pick_up_point)) {
        // Make the marker invisible
        marker.action = visualization_msgs::Marker::DELETE;
        publish_marker("Object being picked up");
        
        // Simulate a pickup
        ROS_INFO("Sleeping for 5 seconds");
        sleep(5);
        robot_state = DROPPING_OFF;
      }
      break;
    }
    case DROPPING_OFF: {
      if (has_reached_point(robot_pose, drop_off_point)) {
          // Publish the marker at the drop off zone
          marker.pose.position.x = drop_off_point.first;
          marker.pose.position.y = drop_off_point.second;
          marker.pose.position.z = 0;

          marker.action = visualization_msgs::Marker::ADD;
          publish_marker("Object delivered to drop off zone");
        
          robot_state = DONE;
      }
      break;
    }
    case DONE: {
      // Do nothing
      break;
    }
  }
 
}

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber amcl_pose_sub = n.subscribe("amcl_pose", 1000, amcl_pose_callback);

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
  
  ros::spin();
  
  return 0;
}