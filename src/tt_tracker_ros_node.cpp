/*
 * tt_tracker_ros_node.cpp
 *
 *  Created on: 1.1.1
 *      Author: Markn West
 *   
 */

#include <tt_tracker_ros/tt_tracker.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tt_tracker_ros");
  ros::NodeHandle nodeHandle("~");
  tt_tracker_ros::tt_tracker ttTracker(nodeHandle);

  ros::spin();
  return 0;
}
