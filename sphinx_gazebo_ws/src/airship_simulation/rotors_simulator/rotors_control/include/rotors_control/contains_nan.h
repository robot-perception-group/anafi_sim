#ifndef CONTAINS_NAN_H
#define CONTAINS_NAN_H

#include <geometry_msgs/PoseStamped.h> // Include the ROS PoseStamped message type

// Function declaration
bool containsNaN(const geometry_msgs::PoseStamped& pose_stamped);

#endif // CONTAINS_NAN_H