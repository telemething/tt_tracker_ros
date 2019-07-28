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
#include <sstream>

#include <boost/interprocess/sync/interprocess_semaphore.hpp>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
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

// tt_tracker_ros_msgs
#include <tt_tracker_ros_msgs/TrackBoxes.h>
#include <tt_tracker_ros_msgs/TrackBox.h>
#include <tt_tracker_ros_msgs/CurrentTrackMode.h>
#include <tt_tracker_ros_msgs/CheckForObjectsAction.h>

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
// List of tracker algorithms in OpenCV 3.4.1
enum trackerAlgEnum {algUninit, BOOSTING, MIL, KCF, TLD,MEDIANFLOW, GOTURN, MOSSE, CSRT};
enum trackerModeEnum {trackUninit, startTracking, tracking, stopTracking, resetTracker};
enum searcherModeEnum {searchUninit, startSearching, searching, found};

class TrackerEngine
{
 public:

  cv::Ptr<cv::Tracker> tracker;
  cv::Rect2d trackingBox;
  bool isOK = true;
  std::string name  = "-";

  trackerModeEnum trackerMode_ = trackerModeEnum::trackUninit;
  searcherModeEnum searcherMode_ = searcherModeEnum::searchUninit;
  trackerAlgEnum trackerAlg = trackerAlgEnum::algUninit;
};


class tt_tracker
{
 public:

  // ROS node handle.
  ros::NodeHandle nodeHandle_;

  // Advertise and subscribe to image topics.
  image_transport::ImageTransport imageTransport_;

  // ROS subscribers 
  image_transport::Subscriber imageSubscriber_;
  ros::Subscriber darknetBoundingBoxesSubscriber_;
  ros::Subscriber gimbalAngleSubscriber_;

  // ROS Publishers
  //ros::Publisher chatter_pub_;
  ros::Publisher TrackBoxesPublisher_;
  ros::Publisher CurrentTackModePublisher_;

  std::string DisplayWindowName_ = "TT Tracker";

  std::string trackerType_;
  std::string searchForClassName_ = "";
  //cv::Ptr<cv::Tracker> tracker_;
  //std::vector<cv::Ptr<cv::Tracker>> trackers_;
  std::vector<TrackerEngine> trackerEngines_;
  cv_bridge::CvImagePtr in_raw_image_;
  cv::Mat out_image_;
  

  std_msgs::Header imageHeader_;

  boost::shared_mutex mutexInputRawImage_;
  boost::shared_mutex mutexOutputImage_;
  boost::shared_mutex mutexDarknetBoundingBoxesCallback_;
  boost::shared_mutex mutexImageStatus_;

  //cv::Rect2d trackingBox_;
  cv::Rect2d trackThisBox_;

  //trackerModeEnum trackerMode_ = trackUninit;
  trackerModeEnum aggregateTrackingMode_ = trackerModeEnum::trackUninit;
  searcherModeEnum searcherMode_ = searcherModeEnum::searchUninit;

  std::thread trackloop_thread; 
  std::thread publishTrackingModeloop_thread;
  std::thread displayloop_thread;

  float trackPerfFps;
  bool trackerOk;
  float foundObjcetClassProbability;
  int frameWidth_;
  int frameHeight_;
  bool imageStatus_ = false;
  bool haveValidGimbalAngle_ = false;
  double lastGoodTrackTickCount_;
  int trackingFailureTimeoutSeconds_ = 1;
  int publishTrackingModeloopTimeoutSeconds_ = 1;
  geometry_msgs::Vector3Stamped gimbalAngle_;
  float minimumObjectIdConfidencePercent;

  tt_tracker_ros_msgs::CurrentTrackMode currentTrackModeMessage_;

  explicit tt_tracker(ros::NodeHandle nh);
  ~tt_tracker();

  struct releativeCoordsStruct
  {
    double hPercent;
    double wPercent;
  };

  bool readParameters();
  int trackloop();
  int publishTrackingModeloop();
  int displayloop();
  void init();
  int addTracker(trackerAlgEnum trackerAlg, std::string name);
  releativeCoordsStruct findPositionRelativeToImageCenter(const cv::Rect objectRect, const int inageSizeH, const int imageSizeW);
  void startSearchingForObject(const std::string className);
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
  void darknetBoundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes& bboxMessage);
  void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
};

} /* namespace darknet_ros*/
