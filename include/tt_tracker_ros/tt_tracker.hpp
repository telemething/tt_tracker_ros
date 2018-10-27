/*
 * tt_tracker.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>

#include <boost/interprocess/sync/interprocess_semaphore.hpp>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>

// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

//---
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
//---

// darknet_ros_msgs
//#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <darknet_ros_msgs/BoundingBox.h>
//#include <darknet_ros_msgs/CheckForObjectsAction.h>

// Darknet.
//#ifdef GPU
//#include "cuda_runtime.h"
//#include "curand.h"
//#include "cublas_v2.h"
//#endif

//Trace
//#define CVVISUAL_DEBUGMODE
#include <opencv2/cvv/debug_mode.hpp>
#include <opencv2/cvv/show_image.hpp>
#include <opencv2/cvv/filter.hpp>
#include <opencv2/cvv/dmatch.hpp>
#include <opencv2/cvv/final_show.hpp>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

namespace tt_tracker_ros 
{

class tt_tracker
{
 public:

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  //! Advertise and subscribe to image topics.
  image_transport::ImageTransport imageTransport_;

    //! ROS subscribers 
  image_transport::Subscriber imageSubscriber_;

  ros::Subscriber darknetBoundingBoxesSubscriber_;

  std::string DisplayWindowName_ = "TT Tracker";

  std::string trackerType_;
  std::string searchForClassName_ = "";
  cv::Ptr<cv::Tracker> tracker_;
  cv_bridge::CvImagePtr in_raw_image_;
  cv::Mat out_image_;
  

  std_msgs::Header imageHeader_;

  boost::shared_mutex mutexInputRawImage_;
  boost::shared_mutex mutexOutputImage_;
  boost::shared_mutex mutexDarknetBoundingBoxesCallback_;
  boost::shared_mutex mutexImageStatus_;

  cv::Rect2d trackingBox_;
  cv::Rect2d trackThisBox_;

  enum trackerModeEnum {trackUninit, startTracking, tracking, stopTracking, resetTracker};
  enum searcherModeEnum {searchUninit, startSearching, searching, found};
  trackerModeEnum trackerMode_ = trackUninit;
  searcherModeEnum searcherMode_ = searchUninit;

  std::thread trackloop_thread; 
  std::thread searchloop_thread;
  std::thread displayloop_thread;

  float trackPerfFps;
  bool trackerOk;
  float foundObjcetClassProbability;
  int frameWidth_;
  int frameHeight_;
  bool imageStatus_ = false;
  double lastGoodTrackTickCount_;
  int trackingFailureTimeoutSeconds = 5;

  //boost::interprocess::interprocess_semaphore cam_image_ready_(0);

  explicit tt_tracker(ros::NodeHandle nh);
  ~tt_tracker();

  bool readParameters();
  int trackloop();
  //int searchloop();
  int displayloop();
  void init();
  int initTracker();
  void startSearchingForObject(const std::string className);
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
  void darknetBoundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes& bboxMessage);
};

} /* namespace darknet_ros*/
