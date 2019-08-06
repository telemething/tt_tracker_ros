/*
 * tt_tracker_ros_node.cpp
 *
 *  Created on: 1.1.1
 *      Author: Markn West
 *   
 */

 // yolo object detector
#include "tt_tracker_ros/tt_tracker.hpp"
#include <boost/format.hpp>
#include <math.h>
#include <csignal>
#include <iostream>

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

boost::interprocess::interprocess_semaphore cam_image_ready_(0);
boost::interprocess::interprocess_semaphore out_image_ready_(0);

//*****************************************************************************
//*
//* Handle sigfaults (from opencv)
//* https://en.cppreference.com/w/cpp/utility/program/signal#Signal_handler
//*
//******************************************************************************

namespace
{
  volatile std::sig_atomic_t gSignalStatus;
}
 
void signal_handler(int signal)
{
  gSignalStatus = signal;
}
 
//*****************************************************************************
//*
//*
//*
//******************************************************************************

namespace tt_tracker_ros 
{

int TrackerEngine::nextIndex = 0;

//*****************************************************************************
//*ww
//*
//*
//******************************************************************************

tt_tracker::tt_tracker(ros::NodeHandle nh) :
      nodeHandle_(nh),
      imageTransport_(nodeHandle_)
{
  ROS_INFO("[tt_tracker] Node started.");

  CreateLogger();
  logger_->info("Node started");

  // Read parameters from config file.
  if (!readParameters()) 
  {
    ros::requestShutdown();
  }

  init();

  startSearchingForObject("person");
  logger_->info("Searching for object (person)");
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

tt_tracker::~tt_tracker()
{
  {
    //boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    //isNodeRunning_ = false;
  }
  //yoloThread_.join();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

bool tt_tracker::readParameters()
{
  // Load common parameters.
  //nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  //nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  //nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

  // Check if Xserver is running on Linux.
  /*if (XOpenDisplay(NULL)) 
  {
    // Do nothing!
    ROS_INFO("[YoloObjectDetector] Xserver is running.");
  } 
  else 
  {
    ROS_INFO("[YoloObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }*/

  return true;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void tt_tracker::init()
{
  //std::signal(SIGINT, signal_handler);
  std::signal(SIGSEGV, signal_handler);

  //std::cout << "SignalValue: " << gSignalStatus << '\n';
  //std::raise(SIGINT);
  //std::cout << "SignalValue: " << gSignalStatus << '\n';

  addTrackerEngine(trackerAlgEnum::MIL, "Mil");
  addTrackerEngine(trackerAlgEnum::CSRT, "Csrt");
  //addTrackerEngine(trackerAlgEnum::GOTURN, "Go");
  addTrackerEngine(trackerAlgEnum::KCF, "Kcf");
  addTrackerEngine(trackerAlgEnum::MEDIANFLOW, "Median");
  addTrackerEngine(trackerAlgEnum::MOSSE, "Mosse");
  addTrackerEngine(trackerAlgEnum::TLD, "Tld");
  addTrackerEngine(trackerAlgEnum::BOOSTING, "Boost");

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  std::string darknetBoundingBoxesName;
  std::string gimbalAngleSubscriberTopicName;
  int cameraQueueSize;  
  int darknetBoundingBoxesQueueSize;

  std::string trackBoxesTopicName;
  int trackBoxesQueueSize;
  bool trackBoxesLatch;

  std::string currentTrackModeTopicName;
  int currentTrackModeQueueSize;
  bool currentTrackModeLatch;

  gimbalAngle_.vector.y = 0;
  gimbalAngle_.vector.x = 0;
  gimbalAngle_.vector.z = 0;
  haveValidGimbalAngle_ = false;

  // Read Config
  //nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
  //                  std::string("/camera/image_raw"));
                    
  //nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
  //                  std::string("/airsim/image_raw"));
                    
  nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
                    std::string("/darknet_ros/detection_image"));
                    
  nodeHandle_.param("darknet_ros/bounding_boxes/topic", darknetBoundingBoxesName,
                    std::string("/darknet_ros/bounding_boxes"));
                    
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);  
  nodeHandle_.param("darknet_ros/bounding_boxes/queue_size", darknetBoundingBoxesQueueSize, 1);

  nodeHandle_.param("subscribers/gimbalAngleRecvd/topic", gimbalAngleSubscriberTopicName,
                    std::string("/tt_gimbal/gimbal_angle_recvd"));


  nodeHandle_.param("publishers/track_boxes/topic", trackBoxesTopicName,
                    std::string("track_boxes"));
  nodeHandle_.param("publishers/track_boxes/queue_size", trackBoxesQueueSize, 1);
  nodeHandle_.param("publishers/track_boxes/latch", trackBoxesLatch, false);

  nodeHandle_.param("publishers/current_track_mode/topic", currentTrackModeTopicName,
                    std::string("current_track_mode"));
  nodeHandle_.param("publishers/current_track_mode/queue_size", currentTrackModeQueueSize, 1);
  nodeHandle_.param("publishers/current_track_mode/latch", currentTrackModeLatch, false);

  nodeHandle_.param("minimumObjectIdConfidencePercent", minimumObjectIdConfidencePercent,
                    float(60.0));

  minimumObjectIdConfidencePercent /= 100;

  //cv::namedWindow(DisplayWindowName_);

  printf("\r\ncameraTopicName: %s\n", cameraTopicName.c_str());
  printf("\r\ndarknetBoundingBoxesName: %s\n", darknetBoundingBoxesName.c_str());
  printf("\r\ntrackBoxesTopicName: %s\n", trackBoxesTopicName.c_str());
  printf("\r\ncurrentTrackModeTopicName: %s\n", currentTrackModeTopicName.c_str());

  // Subscribe

  imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize,
                                               &tt_tracker::cameraCallback, this);

  darknetBoundingBoxesSubscriber_ = 
    nodeHandle_.subscribe(darknetBoundingBoxesName, darknetBoundingBoxesQueueSize,
      &tt_tracker::darknetBoundingBoxesCallback, this);

  gimbalAngleSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Vector3Stamped>
    (gimbalAngleSubscriberTopicName, 10, &tt_tracker::gimbalAngleCallback, this);

  // Publish

  //chatter_pub_ = nodeHandle_.advertise<std_msgs::String>("chatter", 1000);

  TrackBoxesPublisher_ = nodeHandle_.advertise<tt_tracker_ros_msgs::TrackBoxes>(
      trackBoxesTopicName, trackBoxesQueueSize, trackBoxesLatch);

  // string modeName
  // float64 trackSubjectClassName
  CurrentTackModePublisher_ = nodeHandle_.advertise<tt_tracker_ros_msgs::CurrentTrackMode>(
      currentTrackModeTopicName, currentTrackModeQueueSize, false);

  // Start threads                                             
  trackloop_thread = std::thread(&tt_tracker::trackloop, this); 
  publishTrackingModeloop_thread = std::thread(&tt_tracker::publishTrackingModeloop, this); 
  displayloop_thread = std::thread(&tt_tracker::displayloop, this);  
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void tt_tracker::startSearchingForObject(const std::string className)
{
  searchForClassName_ = className;

  for( auto &trackerEngine : trackerEngines_)
  {
    trackerEngine.trackerMode_ = trackerModeEnum::stopTracking;
    trackerEngine.searcherMode_ = searcherModeEnum::searching;
  }

  aggregateSearchingMode_ = searcherModeEnum::searching;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void tt_tracker::gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  gimbalAngle_ = *msg;
  haveValidGimbalAngle_ = true;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void tt_tracker::darknetBoundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes& bboxMessage)
{
  //ROS_DEBUG("[tt_tracker] darknetBoundingBoxes received.");
  //logger_->debug("darknetBoundingBoxes received.");

  if(aggregateSearchingMode_ != searching)
    return;

  // try to lock this section
  boost::shared_lock<boost::shared_mutex> lock(mutexDarknetBoundingBoxesCallback_, boost::try_to_lock);

  // if we cant lock it then just drop this message instance
  if(!lock)
    return;

  // can't search for an empty searchForClassName_
  if(searchForClassName_ == "")
    return;

  try 
  {
    unsigned int Seq = bboxMessage.header.seq;

  	std::vector<darknet_ros_msgs::BoundingBox> bbox = bboxMessage.bounding_boxes;

    // Very verbose, only do this while debugging this one thing
    if(printDetectedObjectNames_)
    {
      logger_->debug("------ Seq:{0:d}",Seq);
      logger_->debug("Objects:");
    }

    for(int index = 0; index < bbox.size(); index++)
    {
      // Very verbose, only do this while debugging this one thing
      if(printDetectedObjectNames_)
        logger_->debug("{}", bbox[index].Class.c_str());

      if( bbox[index].Class.c_str() == searchForClassName_)
      {
        foundObjcetClassProbability = bbox[index].probability;    
        trackThisBox_.x = bbox[index].xmin;
        trackThisBox_.y = bbox[index].ymin;
        trackThisBox_.width = bbox[index].xmax - bbox[index].xmin;
        trackThisBox_.height = bbox[index].ymax - bbox[index].ymin;

        //for each tracker engine
        for( auto &trackerEngine : trackerEngines_)
        {
          //if it is not tracking, then start tracking
          if(trackerEngine.trackerMode_ != trackerModeEnum::tracking)
            trackerEngine.trackerMode_ = trackerModeEnum::startTracking;
        }

        return;
      }
    }  
  } 
  catch ( ... ) 
  {
    ROS_ERROR("darknetBoundingBoxesCallback exception");
    logger_->error("darknetBoundingBoxesCallback exception");
    return;
  }
 
  return;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int cvvCount = 5;

void tt_tracker::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("[tt_tracker] image received.");
  //logger_->debug("image received.");

  try 
  {
    
    boost::shared_lock<boost::shared_mutex> lock(mutexInputRawImage_, boost::try_to_lock);

    // if we cant lock it then just drop this frame
    if(!lock)
      return;

    in_raw_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    //TEST
    //imshow(DisplayWindowName_, cam_image_->image);

    imageHeader_ = msg->header;   

    if (in_raw_image_) 
    {
      //TEST
      //cv::imshow(DisplayWindowName_, cam_image_->image);
      //cv::waitKey(3);

      frameWidth_ = in_raw_image_->image.size().width;
      frameHeight_ = in_raw_image_->image.size().height;

      cam_image_ready_.post();
    }

  } 
  catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("-- EXCEPTION --- tt_tracker::cameraCallback: %s", e.what());
    logger_->error("- EXCEPTION --- tt_tracker::cameraCallback: {}", e.what());
    return;
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("--- EXCEPTION --- tt_tracker::cameraCallback: %s", e.what());
    logger_->error("- EXCEPTION --- tt_tracker::cameraCallback: {}", e.what());
  }
  catch(...)
  {
    ROS_ERROR("--- EXCEPTION --- tt_tracker::cameraCallback: -undefined-");
    logger_->error("--- EXCEPTION --- tt_tracker::cameraCallback: -undefined-");
  }

  
  return;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

cv::Ptr<cv::Tracker> tt_tracker::CreateTracker(trackerAlgEnum trackerAlg)
{
  switch(trackerAlg)
  {
    case BOOSTING:
      return cv::TrackerBoosting::create();
      break;
    case MIL:
      return cv::TrackerMIL::create();
       break;
    case KCF:
       return cv::TrackerKCF::create();
       break;
    case TLD:
       return cv::TrackerTLD::create();
       break;
    case MEDIANFLOW:
       return cv::TrackerMedianFlow::create();
       break;
    case GOTURN:
       return cv::TrackerGOTURN::create();
       break;
    case MOSSE:
       return cv::TrackerMOSSE::create();
       break;
    case CSRT:
       return cv::TrackerCSRT::create();
       break;
  }

  //TODO : throw if no match
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int tt_tracker::addTrackerEngine(trackerAlgEnum trackerAlg, std::string name)
{
  logger_->debug("-CREATE Tracker: {}", name.c_str());

  TrackerEngine trackerEngine;
  bool foundTrackerEngine = false;

  // for now we only want one tracker of each algorithm
  // so if we find one then we use it. We release the
  // existing cv tracker because we want a new one
  for( auto &tEngine : trackerEngines_)
    if(tEngine.trackerAlg == trackerAlg)
    {
      foundTrackerEngine = true;
      trackerEngine.tracker.release();
      trackerEngine = tEngine;
    }

  trackerEngine.name = name;
  trackerEngine.trackerAlg = trackerAlg;
  trackerEngine.hasData = false;
  trackerEngine.isInStdDev = false;
  trackerEngine.isOK = true;
  trackerEngine.searcherMode_ = searcherModeEnum::searchUninit;
  trackerEngine.trackerMode_  = trackerModeEnum::trackUninit;
  trackerEngine.tracker = CreateTracker(trackerEngine.trackerAlg);

  if(!foundTrackerEngine)
  {
    trackerEngine.index = TrackerEngine::nextIndex++;
    trackerEngines_.push_back(trackerEngine);
  }

  // we create these globally because of very high use
  auto engineCount = trackerEngines_.size();
  _distMatrix = new float[engineCount,engineCount];
  _distSumMatrix = new float[engineCount];

  return 0;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int tt_tracker::resetTrackerEngine(TrackerEngine& trackerEngine)
{
  logger_->debug("-RESET : {}", trackerEngine.name.c_str());

  //trackerEngine.tracker->clear();

  //exclusive access required because releasing and creating
  boost::shared_lock<boost::shared_mutex> lock(*(trackerEngine.mutexCvTracker));
  logger_->debug("-RESET : {} : Lock aquired", trackerEngine.name.c_str());

  trackerEngine.tracker.release();
  trackerEngine.hasData = false;
  trackerEngine.isInStdDev = false;
  trackerEngine.isOK = true;
  trackerEngine.trackerMode_ = trackerModeEnum::startTracking;
  trackerEngine.searcherMode_ = searcherModeEnum::searchUninit;
  trackerEngine.tracker = CreateTracker(trackerEngine.trackerAlg);

  //explicity unlock so that we can confidently write debug message
  lock.release();
  logger_->debug("-RESET : {} : Lock released", trackerEngine.name.c_str());

  return 0;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int tt_tracker::publishTrackingModeloop()
{ 
    while(true)
    {         
      //lock ?
      //boost::shared_lock<boost::shared_mutex> lock(mutexInputRawImage_);

      std::this_thread::sleep_for(std::chrono::seconds(publishTrackingModeloopTimeoutSeconds_));

      currentTrackModeMessage_.trackSubjectClassName = searchForClassName_;

      switch(aggregateSearchingMode_)
      {
        case searchUninit:
          currentTrackModeMessage_.modeName = "Idle"; 
        break;        
        
        case startSearching:
          currentTrackModeMessage_.modeName = "Searching"; 
        break;        
        
        case searching:
          currentTrackModeMessage_.modeName = "Searching"; 
        break;        

        case found:
          switch(aggregateTrackingMode_)
          {
            case trackUninit:
              currentTrackModeMessage_.modeName = "Idle";       
            break;

            case startTracking:
              currentTrackModeMessage_.modeName = "Tracking";
            break;

            case tracking:
              currentTrackModeMessage_.modeName = "Tracking";
            break;


            case stopTracking:
              currentTrackModeMessage_.modeName = "Idle";
            break;

            case resetTracker:
              currentTrackModeMessage_.modeName = "Reseting";
            break;
          }

        break;
      }

      CurrentTackModePublisher_.publish(currentTrackModeMessage_);
    }

  return 0;
}

//*****************************************************************************
//*
//* find the engine nearest to the highest density of points
//*
//******************************************************************************

/*/void tt_tracker::calcD()
{
  // TODO : this only works if two or more points are tracking
  // do things like compare to detector and history of accuracy

  // how many engines do we have?
  auto engineCount = trackerEngines_.size();

  int smallestEngineSumValue = INT_MAX;
  int smallestEngineSumIndex = 0;
  double engineVariance;
  float engineStdDev;
  int engineOkCount;
  double engineMean;

  //build an array of distances
  for(int a = 0; a < engineCount; a++)
  {
    // engine not ok or has no data, blank out entire row
    if(!trackerEngines_[a].isOK | !trackerEngines_[a].hasData)
    {
      for(int b = 0; b < engineCount; b++)
        _distMatrix[a,b] = 0.0f;
      continue;
    }

    // we calculate the sum of distances per engine
    _distSumMatrix[a] = 0.0f;
    engineVariance = 0.0f;
    engineOkCount = 0;

    for(int b = 0; b < engineCount; b++)
    {
      // if either engine is not ok then it's entry shouldn't count
      if(!trackerEngines_[b].isOK)
        _distMatrix[a,b] = 0.0f;
      else
      {
        _distMatrix[a,b] = 
          pow(trackerEngines_[a].centerOfMass.x - trackerEngines_[b].centerOfMass.x, 2) + 
          pow(trackerEngines_[a].centerOfMass.y - trackerEngines_[b].centerOfMass.y, 2);

        _distSumMatrix[a] += _distMatrix[a,b];
        engineOkCount++;
      }
    }

    if(engineOkCount == 0)
    {
      // TODO : Do something
    }
    else if(engineOkCount == 1)
    {
      // TODO : Do something
    }
    else
    {
      // find the mean
      engineMean = _distSumMatrix[a] / (float)engineOkCount;

      // find the variance
      for(int b = 0; b < engineCount; b++)
        engineVariance += pow(_distMatrix[a,b] - engineMean, 2);

      // find the stddev
      engineStdDev = sqrt(engineVariance);

      // remember which engine has the lowest sum of distances
      if(_distSumMatrix[a] <  smallestEngineSumValue)
      {
        smallestEngineSumValue = _distSumMatrix[a];
        smallestEngineSumIndex = a;
      }

      _currentBestTracker = smallestEngineSumIndex;

      // loop through the row again and identify the ones out of stddev
      for(int b = 0; b < engineCount; b++)
      {
        if(_distMatrix[a,b] < engineStdDev )
          trackerEngines_[b].isInStdDev = true;
        else
          trackerEngines_[b].isInStdDev = false;
      }
    }
  } 
}*/

void tt_tracker::calcD()
{
  // TODO : this only works if two or more points are tracking
  // do things like compare to detector and history of accuracy

  // how many engines do we have?
  auto engineCount = trackerEngines_.size();

  int smallestEngineSumValue = INT_MAX;
  int smallestEngineSumIndex = 0;
  int largestEngineSumValue = 0;
  int largestEngineSumIndex = 0;
  double engineVariance;
  float engineStdDev;
  int engineOkCount;
  double engineMean;

  double sum = 0;
  double mean = 0;
  double variance = 0;
  double stdDev = 0;

  //build an array of distances
  for(int a = 0; a < engineCount; a++)
  {
    // engine not ok or has no data, blank out entire row
    if(!trackerEngines_[a].isOK | !trackerEngines_[a].hasData)
    {
      for(int b = 0; b < engineCount; b++)
        _distMatrix[a,b] = 0.0f;
      continue;
    }

    // we calculate the sum of distances per engine
    _distSumMatrix[a] = 0.0f;
    engineVariance = 0.0f;
    engineOkCount = 0;

    for(int b = 0; b < engineCount; b++)
    {
      // if either engine is not ok then it's entry shouldn't count
      if(!trackerEngines_[b].isOK)
        _distMatrix[a,b] = 0.0f;
      else
      {
        _distMatrix[a,b] = sqrt(
          pow(trackerEngines_[a].centerOfMass.x - trackerEngines_[b].centerOfMass.x, 2) + 
          pow(trackerEngines_[a].centerOfMass.y - trackerEngines_[b].centerOfMass.y, 2));

        _distSumMatrix[a] += _distMatrix[a,b];
        engineOkCount++;
      }
    }
    
    if(engineOkCount == 0)
    {
      //this shouldnt be possible
      logger_->error("--- ERROR --- calcD() : Inner engineOkCount == 0");

      // set this out of bounds so later procs dont find it
      smallestEngineSumIndex = -1;
      largestEngineSumIndex = -1;    
    }
    else if(engineOkCount == 1)
    {
      // the only viable engine is the one in the outer loop
      smallestEngineSumIndex = a;
      // set this out of bounds so later procs dont find it
      largestEngineSumIndex = -1;    
    }
    else
    {
      // find the mean
      /*engineMean = _distSumMatrix[a] / (float)engineOkCount;

      // find the variance
      for(int b = 0; b < engineCount; b++)
        engineVariance += pow(_distMatrix[a,b] - engineMean, 2);

      // find the stddev
      engineStdDev = sqrt(engineVariance);*/

      // remember which engine has the lowest sum of distances
      if(_distSumMatrix[a] <  smallestEngineSumValue)
      {
        smallestEngineSumValue = _distSumMatrix[a];
        smallestEngineSumIndex = a;
      }

      // remember which engine has the highest sum of distances
      if(_distSumMatrix[a] > largestEngineSumValue)
      {
        largestEngineSumValue = _distSumMatrix[a];
        largestEngineSumIndex = a;
      }
    }

    sum += _distSumMatrix[a];
  }

  _currentBestTracker = smallestEngineSumIndex;
  _currentWorstTracker = largestEngineSumIndex;

  mean = sum/(float)engineOkCount;

  for(int a = 0; a < engineCount; a++)
  {
    if(trackerEngines_[a].isOK | trackerEngines_[a].hasData)
      variance += pow(_distSumMatrix[a] - mean, 2);
  }

  variance /= engineOkCount;
  stdDev = sqrt(variance);

  // we want to mark it as out if stdDev if it is X times stdDev
  //if( (stdDev * 5) > largestEngineSumValue )
  //  _currentWorstTracker = -1;  

  // TODO : For now lets just say the worst tracker is the only one out
  for(int a = 0; a < engineCount; a++)
  {
    trackerEngines_[a].distance = _distSumMatrix[a];
    trackerEngines_[a].stdDistance = stdDev;

    if(a == _currentWorstTracker)
      trackerEngines_[a].isInStdDev = false;
    else
      trackerEngines_[a].isInStdDev = true;
  }


      /*_currentBestTracker = smallestEngineSumIndex;

      // loop through the row again and identify the ones out of stddev
      for(int b = 0; b < engineCount; b++)
      {
        if(_distMatrix[a,b] < engineStdDev )
          trackerEngines_[b].isInStdDev = true;
        else
          trackerEngines_[b].isInStdDev = false;
      }
    }
  } */
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int tt_tracker::logloop()
{
  while(true)
  {
    if(NULL != logger_)
      logger_->flush();

    std::this_thread::sleep_for(std::chrono::milliseconds(logloopTimeoutMilliseconds_));
  }
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int tt_tracker::trackloop()
{ 
    double isOkTimeoutTicks = cv::getTickFrequency() * isOkTimeoutSeconds_;
    double isInStdDevTimeoutTicks = cv::getTickFrequency() * isInStdDevTimeoutSeconds_;
    double timer;

    while(true)
    {     
      //wait for an image to appear
      cam_image_ready_.wait();

      //just loop (avoid locking) if we dont have work
      //if(trackerMode_ != startTracking & trackerMode_ != tracking)
      //  continue;

      //ROS_DEBUG("[tt_tracker] image received.");
     
      //lock the image so new images don't overwrite
      boost::shared_lock<boost::shared_mutex> lock(mutexInputRawImage_);
      
      // TODO : Remove this, it is just for comparison
      //cv::Rect2d beforeBox;

      logger_->debug("--- trackloop ---");
      
      for( TrackerEngine &trackerEngine : trackerEngines_)
      {
        try
        {   
          //printf("------ Engine : %s ------\n", trackerEngine.name.c_str() );

          //try to lock cvTracker (to prevent clash with other threads)
          boost::shared_lock<boost::shared_mutex> trackerLock(*(trackerEngine.mutexCvTracker), boost::try_to_lock);

          // if we cant lock it then just drop this frame
          if(!trackerLock)
            continue;

          // Start timer
          timer = (double)cv::getTickCount();

          switch(trackerEngine.trackerMode_)
          {
            case trackerModeEnum::tracking:

              
              // TODO : Remove this, it is just for comparison
              //beforeBox.x = trackerEngine.trackingBox.x;
              //beforeBox.y = trackerEngine.trackingBox.y;
              //beforeBox.height = trackerEngine.trackingBox.height;
              //beforeBox.width = trackerEngine.trackingBox.width;

              /*if(trackerEngine.trackingBox.x < 0.0)
              {
                logger_->debug("^ trackingBox.x < 0.0 : {}", trackerEngine.name.c_str());
                trackerEngine.trackerMode_ = trackerModeEnum::resetTracker;	
                continue;
              }

              if(trackerEngine.trackingBox.y < 0.0)
              {
                logger_->debug("^ trackingBox.y < 0.0 : {}", trackerEngine.name.c_str());
                trackerEngine.trackerMode_ = trackerModeEnum::resetTracker;	
                continue;
              }*/

              trackerEngine.isOK = trackerEngine.tracker->update(in_raw_image_->image, trackerEngine.trackingBox);
              trackerEngine.centerOfMass = (trackerEngine.trackingBox.br() + trackerEngine.trackingBox.tl())*0.5;
              trackerEngine.hasData = true;

              // Calculate Frames per second (FPS)
              trackPerfFps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

              // Manage isOk
              if(trackerEngine.isOK)
              {
                  // Get a timestamp
                  trackerEngine.lastIsOkTickCount = timer;
              }
              else 
              {
                // If we have lost track for x amount of time then re initiate search mode
                if( (timer - trackerEngine.lastIsOkTickCount) > isOkTimeoutTicks )
                {
                  logger_->debug("^ isOK Out : {}", trackerEngine.name.c_str() );	
                  trackerEngine.trackerMode_ = trackerModeEnum::resetTracker;  
                }
              } 
                // Manage isStdDev
                if(trackerEngine.isInStdDev)
                {
                    // Get a timestamp
                    trackerEngine.lastInStdDevTickCount = timer;
                }
                else
                {
                  // If we are out of stddev for x amount of time then re initiate search mode
                  if( (timer - trackerEngine.lastInStdDevTickCount) > isInStdDevTimeoutTicks )
                  {
                    logger_->debug("^ StdDev Out : {} - dist: {:.0f} - stdDev: {:.0f}", 
                      trackerEngine.name.c_str(), trackerEngine.distance, trackerEngine.stdDistance );	
                    trackerEngine.trackerMode_ = trackerModeEnum::resetTracker;  
                  }
                }

            break;

            case trackerModeEnum::startTracking:

              trackerEngine.trackingBox.x = trackThisBox_.x;
              trackerEngine.trackingBox.y = trackThisBox_.y;
              trackerEngine.trackingBox.width = trackThisBox_.width;
              trackerEngine.trackingBox.height = trackThisBox_.height;
              trackerEngine.isOK = trackerEngine.tracker->init(in_raw_image_->image, trackerEngine.trackingBox);

              logger_->debug("start tracking : {} - xywh: {:.0f}:{:.0f}:{:.0f}:{:.0f} - ok: {}", 
                trackerEngine.name.c_str(), 
                trackerEngine.trackingBox.x, trackerEngine.trackingBox.y, 
                trackerEngine.trackingBox.width, trackerEngine.trackingBox.height,
                trackerEngine.isOK );	

              trackerEngine.trackerMode_ = trackerModeEnum::tracking;
              trackerEngine.hasData = false;
              trackerEngine.isInStdDev = false;
                                
              // init these here so that later tests have a baseline even 
              // if never ok or in stddev
              trackerEngine.lastIsOkTickCount = timer;
              trackerEngine.lastInStdDevTickCount = timer;
              
            break;

            case trackerModeEnum::resetTracker:

              logger_->debug("reset tracking : {}", trackerEngine.name.c_str() );	

              // TODO : maybe we shouldn't reinit
              trackerEngine.trackerMode_ = trackerModeEnum::startTracking;
              resetTrackerEngine(trackerEngine);
              trackerEngine.searcherMode_ = searcherModeEnum::searching;

            break;
          } // switch(trackerEngine.trackerMode_)
        }
        catch(const std::exception& e)
        {
          logger_->error("--- EXCEPTION --- : tracker : {0} : {1}", trackerEngine.name.c_str(), e.what());
          ROS_ERROR("--- EXCEPTION --- trackloop switch(trackerEngine.trackerMode_): %s", e.what());
        }
        catch(...)
        {
          logger_->error("--- EXCEPTION --- : tracker : {0} : -undefined-", trackerEngine.name.c_str());
          ROS_ERROR("--- EXCEPTION --- trackloop switch(trackerEngine.trackerMode_): -undefined-");
        }

      } // for( auto &trackerEngine : trackerEngines_)

      calcD();

      //is anybody tracking? Other threads are running, so dont use 
      //aggregateTrackingMode_ as a temp. Only change it once as needed.
      bool somebodyIsTracking = false;
      bool somebodyIsSearching = false;

      for( auto &trackerEngine : trackerEngines_)
      {
        if(trackerEngine.trackerMode_ == trackerModeEnum::tracking)
          somebodyIsTracking = true;
        if(trackerEngine.searcherMode_ == searcherModeEnum::searching)
          somebodyIsSearching = true;
      }

      if(somebodyIsTracking)
        aggregateTrackingMode_ = trackerModeEnum::tracking;
      else
        aggregateTrackingMode_ = trackerModeEnum::trackUninit;

      if(somebodyIsSearching)
        aggregateSearchingMode_ = searcherModeEnum::searching;
      else
        aggregateSearchingMode_ = searcherModeEnum::searchUninit;

      //try to lock image, don't get hung up
      boost::shared_lock<boost::shared_mutex> outlock(mutexOutputImage_, boost::try_to_lock);

      // if we cant lock it then just drop this frame
      if(!outlock)
        continue;

      in_raw_image_->image.copyTo(out_image_);
      out_image_ready_.post();
    }

  return 0;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

tt_tracker::releativeCoordsStruct tt_tracker::findPositionRelativeToImageCenter(
  const cv::Rect objectRect, const int imageSizeH, const int imageSizeW)
{
  releativeCoordsStruct releativeCoords;
  releativeCoords.hPercent = 0;
  releativeCoords.wPercent = 0;

  cv::Point objectCOM = (objectRect.br() + objectRect.tl())*0.5;

  double dX = -((double)objectCOM.x - (double)imageSizeW / 2);
  double dY = (double)objectCOM.y - (double)imageSizeH / 2;

  releativeCoords.hPercent = dY / imageSizeH;
  releativeCoords.wPercent = dX / imageSizeW;

  return releativeCoords;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

cv::Scalar tt_tracker::GetStatusColor(TrackerEngine& trackerEngine)
{
  cv::Scalar statusColor;

  if(trackerEngine.index == _currentBestTracker)
    statusColor = cv::Scalar(0,255,0); // green
  else if(!trackerEngine.hasData)
  {
    statusColor = cv::Scalar(128,128.128); //gray
    //InfoText = "no data";
  }
  else if(!trackerEngine.isOK)
  {
    statusColor = cv::Scalar(0,0,255); //red
    //InfoText = "bad";
  }
  else if(!trackerEngine.isInStdDev)
  {
    statusColor = cv::Scalar(0,255,255); // yellow
    //InfoText = "out";
  }
  else 
  {
    statusColor = cv::Scalar(255,0,0); // blue
    //InfoText = "";
  }

  return statusColor;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int tt_tracker::displayloop()
{ 
    cv::namedWindow(DisplayWindowName_);
    int count = 0;
    tt_tracker::releativeCoordsStruct releativeCoords;
    //cv::Point objectCOM;
    cv::Point IamgeCOM(frameWidth_/2,frameHeight_/2);
    cv::Scalar lineColor;
    int hPos = 0;

    while(true)
    {     
      //wait for an image to appear
      out_image_ready_.wait();

      //just loop (avoid locking) if we dont have work
      //if(trackerMode_ != startTracking & trackerMode_ != tracking)
      //  continue;

      //ROS_DEBUG("[tt_tracker] output image ready.");
    
      //lock the image so new images don't overwrite
      boost::shared_lock<boost::shared_mutex> lock(mutexOutputImage_);

      hPos = 20;

      //Show searcher mode if appropriate
      switch(aggregateSearchingMode_)
      {
        case searching:
          //putText(out_image_, "Searching for: " + searchForClassName_ , cv::Point(50,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);

          // Display tracker type on frame
          putText(out_image_, 
            str(boost::format("Subject: %s") % searchForClassName_ ), 
            cv::Point(10,hPos += 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50,170,50),1);
          
          // Display gimbal orientation on frame
          /*if(haveValidGimbalAngle_)
            putText(out_image_, 
              str(boost::format("Gimbal p: %.1f r: %.1f y: %.1f") % gimbalAngle_.vector.y % gimbalAngle_.vector.x % gimbalAngle_.vector.z), 
              cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50,170,50),1);
          else
            putText(out_image_, "Gimbal : Unavailable",
              cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50,170,50),1);*/

        break;

        case searchUninit:
          putText(out_image_, "Not searching" , cv::Point(100,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
        break;
      }    
     
      switch(aggregateTrackingMode_)
      {
        case startTracking:

        break;

        case tracking:
         
          for( auto &trackerEngine : trackerEngines_)
          {
            // Tracking success : Draw the tracked object
            //rectangle(out_image_, trackingBox_, cv::Scalar( 255, 0, 0 ), 2, 1 );

            if (trackerEngine.isOK)
            {
              rectangle(out_image_, trackerEngine.trackingBox, cv::Scalar( 255, 0, 0 ), 2, 1 );

              // Draw delta line               
              line(out_image_, IamgeCOM, trackerEngine.centerOfMass, GetStatusColor(trackerEngine), 2, 1 );

              putText(out_image_, 
              str(boost::format("%d") % trackerEngine.index), 
                trackerEngine.centerOfMass, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50,170,50),1);

              // TODO : This is the ROS publisher, it does not belog in the display loop
              /*releativeCoords = findPositionRelativeToImageCenter(trackerEngine.trackingBox, frameHeight_, frameWidth_ );

              tt_tracker_ros_msgs::TrackBoxes TrackBoxesResults_;
              tt_tracker_ros_msgs::TrackBox TrackBox;

              TrackBox.Class = searchForClassName_;
              TrackBox.probability = foundObjcetClassProbability;
              TrackBox.xmin = trackerEngine.trackingBox.x;
              TrackBox.ymin = trackerEngine.trackingBox.y;
              TrackBox.xmax = trackerEngine.trackingBox.width;
              TrackBox.ymax = trackerEngine.trackingBox.height;

              TrackBox.coordsPercentH = releativeCoords.hPercent;
              TrackBox.coordsPercentW = releativeCoords.wPercent;

              TrackBox.action = "TrackThis";
              TrackBoxesResults_.header.stamp = ros::Time::now();
              TrackBoxesResults_.header.frame_id = "tracking";
                
              TrackBoxesResults_.track_boxes.push_back(TrackBox);
              TrackBoxesPublisher_.publish(TrackBoxesResults_);*/
            }
            else
            {
              // Tracking failure detected.
              //TODO Do we really need this?
              //putText(out_image_, "Tracking failure detected", 
              //  cv::Point(100,80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255),2);
            }
          }

          // Draw circle in center of image               
          circle(out_image_, IamgeCOM, 10, cv::Scalar( 0, 0, 255 ), 2, 1 );

          // Display tracker type on frame
          putText(out_image_, 
            str(boost::format("%s(%s) %.2f%% : %s") 
              % currentTrackModeMessage_.modeName % searchForClassName_ 
              % foundObjcetClassProbability % SSTR(int(trackPerfFps))), 
            cv::Point(10,hPos += 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50,170,50),1);
         
          // Display offset on frame
          putText(out_image_, 
            str(boost::format("w: %.3f h: %.3f") % releativeCoords.wPercent % releativeCoords.hPercent), 
            cv::Point(10,hPos += 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50,170,50),1);

          // Display gimbal orientation on frame
          if(haveValidGimbalAngle_)
          putText(out_image_, 
            str(boost::format("Gimbal p: %.1f r: %.1f y: %.1f") 
              % gimbalAngle_.vector.y % gimbalAngle_.vector.x % gimbalAngle_.vector.z), 
            cv::Point(10,hPos += 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50,170,50),1);
          else
            putText(out_image_, "Gimbal : Unavailable",
              cv::Point(10,hPos += 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50,170,50),1);

          // show tracker stats on image
          std::string EngineText;
          std::string InfoText = "";

          for( auto &trackerEngine : trackerEngines_)
          {
            /*if(!trackerEngine.hasData)
            {
              InfoText = "no data";
            }
            else if(!trackerEngine.isOK)
            {
              InfoText = "bad";
            }
            else if(!trackerEngine.isInStdDev)
            {
              InfoText = "out";
            }
            else 
            {
              InfoText = "";
            }*/

            EngineText = str(boost::format("%d %s %s") 
                % trackerEngine.index % trackerEngine.name % InfoText );

            putText(out_image_, EngineText,
                cv::Point(10,frameHeight_ - 10 - trackerEngine.index * 12), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, GetStatusColor(trackerEngine),1);
          }
           
        break;
      }

      try
      {
        // Display frame.
        cv::imshow(DisplayWindowName_, out_image_);
      }
      catch(const std::exception& e)
      {
        logger_->debug("--- EXCEPTION --- displayloop : {}",  e.what());	
        ROS_ERROR("--- EXCEPTION --- ccv::imshow: %s", e.what());
      }
      catch(...)
      {
        logger_->debug("--- EXCEPTION --- displayloop : -undefined-");
        ROS_ERROR("--- EXCEPTION --- ccv::imshow: -undefined-");
      }
                
      // Exit if ESC pressed.
      int k = cv::waitKey(1);
      
      if(k == 27)
      {
        break;
      }
    }

    return 0;
  }
  
  //*****************************************************************************
  //*
  //* decsription: Create two log sinks.
  //*   console : warnings and above
  //*   file : trace (everything) and above
  //* info: https://github.com/gabime/spdlog
  //* examples:
  //*   logger.set_level(spdlog::level::debug);
  //*   spdlog::info("Welcome to spdlog!");
  //*   spdlog::error("Some error message with arg: {}", 1);
  //*   spdlog::warn("Easy padding in numbers like {:08d}", 12);
  //*   spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
  //*   spdlog::info("Support for floats {:03.2f}", 1.23456);
  //*   spdlog::info("Positional args are {1} {0}..", "too", "supported");
  //*   spdlog::info("{:<30}", "left aligned");
  //* Change log pattern
  //*   spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");
  //* Compile time log levels, define SPDLOG_ACTIVE_LEVEL to desired level
  //*   SPDLOG_TRACE("Some trace message with param {}", {});
  //*   SPDLOG_DEBUG("Some debug message");
  //*
  //******************************************************************************

  void tt_tracker::CreateLogger()
  {
    spdlog::info("Creating logs");

    try 
    {
      time_t rawtime;
      struct tm * timeinfo;
      char buffer[80];
      spdlog::level::level_enum logLevel_ = spdlog::level::debug;
      std::string logLevelName = "---";

      // we need to create a log flush thread, because the one built in to spdlog
      // doesn't work
      log_thread = std::thread(&tt_tracker::logloop, this); 

      // create time string
      time (&rawtime);
      timeinfo = localtime(&rawtime);
      strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
      std::string logDirectory = "/Data/Shared/Logs/";
      auto fileName = logDirectory + "TTTracker_Log_" + std::string(buffer) + ".txt";

      // create console logger
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(spdlog::level::warn);
      console_sink->set_pattern("[%H:%M:%S][%t][%^%L%$] %v");
      
      // create file logger
      auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(fileName, true);
      file_sink->set_level(spdlog::level::trace);
      file_sink->set_pattern("[%H:%M:%S][%t][%^%L%$] %v");

      // create multi logger
      spdlog::sinks_init_list sl = {console_sink, file_sink};
      logger_ = std::make_shared<spdlog::logger>("TTTracker", sl);

      switch(logLevel_)
      {
        case spdlog::level::off : logLevelName = "off"; break;
        case spdlog::level::trace : logLevelName = "trace"; break;
        case spdlog::level::debug : logLevelName = "debug"; break;
        case spdlog::level::info : logLevelName = "info"; break;
        case spdlog::level::warn : logLevelName = "warn"; break;
        case spdlog::level::err : logLevelName = "err"; break;
        case spdlog::level::critical : logLevelName = "critical"; break;
      }

      // set level low to show info on startup
      logger_->set_level(spdlog::level::trace);
      spdlog::info("log file created: {0}, level: {1}", fileName, logLevelName);

      // set level to normal going forward
      logger_->set_level(logLevel_);
      logger_->warn("this should appear in both console and file");
      logger_->info("this message should not appear in the console, only in the file");

      // these dont seem to work, but maybe someday they will
      logger_->flush_on(spdlog::level::err);
      spdlog::flush_on(spdlog::level::err);
    }
    catch (const spdlog::spdlog_ex &ex)
    {
      std::cout << "Log init failed: " << ex.what() << std::endl;
            
      // create console logger
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(spdlog::level::warn);
      console_sink->set_pattern("[%H:%M:%S][%t][%^%L%$] %v");

      spdlog::sinks_init_list sl = {console_sink};
      logger_ = std::make_shared<spdlog::logger>("TTTracker", sl);
    }     
  }
}
