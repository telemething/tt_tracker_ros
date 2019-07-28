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

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

boost::interprocess::interprocess_semaphore cam_image_ready_(0);
boost::interprocess::interprocess_semaphore out_image_ready_(0);

//*****************************************************************************
//*
//*
//*
//******************************************************************************

namespace tt_tracker_ros 
{

//*****************************************************************************
//*
//*
//*
//******************************************************************************

tt_tracker::tt_tracker(ros::NodeHandle nh) :
      nodeHandle_(nh),
      imageTransport_(nodeHandle_)
{
  ROS_INFO("[tt_tracker] Node started.");

  // Read parameters from config file.
  if (!readParameters()) 
  {
    ros::requestShutdown();
  }

  init();

  startSearchingForObject("person");
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

  ROS_INFO("[tt_tracker] X--- sup ---");

  return true;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void tt_tracker::init()
{
  ROS_INFO("[tt_tracker] init().");

  addTracker(trackerAlgEnum::MIL, "MIL");

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
  //trackerMode_ = stopTracking;

  for( auto &trackerEngine : trackerEngines_)
    trackerEngine.trackerMode_ = trackerModeEnum::stopTracking;

  searchForClassName_ = className;
  searcherMode_ = searcherModeEnum::searching;
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
  ROS_DEBUG("[tt_tracker] darknetBoundingBoxes received.");

  if(searcherMode_ != searching)
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

    printf("\n\n------ Seq:%.u\n",Seq);
    printf("Objects:\n\n");

    for(int index = 0; index < bbox.size(); index++)
    {
      printf("%s\n", bbox[index].Class.c_str() );	

      if( bbox[index].Class.c_str() == searchForClassName_)
      {
        foundObjcetClassProbability = bbox[index].probability;    
        trackThisBox_.x = bbox[index].xmin;
        trackThisBox_.y = bbox[index].ymin;
        trackThisBox_.width = bbox[index].xmax - bbox[index].xmin;
        trackThisBox_.height = bbox[index].ymax - bbox[index].ymin;

        searcherMode_ = found;

        //trackerMode_ = startTracking;

        //for each tracker engine
        for( auto &trackerEngine : trackerEngines_)
        {
          //if it is not tracking, then start tracking
          //if(trackerEngine.trackerMode_ != trackerModeEnum::tracking)
            trackerEngine.trackerMode_ = trackerModeEnum::startTracking;
        }

        return;
      }
    }  
  } 
  catch ( ... ) 
  {
    ROS_ERROR("darknetBoundingBoxesCallback exception");
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
  } 
  catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (in_raw_image_) 
  {
    //TEST
    //cv::imshow(DisplayWindowName_, cam_image_->image);
    //cv::waitKey(3);

    frameWidth_ = in_raw_image_->image.size().width;
    frameHeight_ = in_raw_image_->image.size().height;

    cam_image_ready_.post();
  }
  
  return;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int tt_tracker::addTracker(trackerAlgEnum trackerAlg, std::string name)
{
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

  trackerEngine.isOK = true;
  trackerEngine.name = name;
  trackerEngine.searcherMode_ = searcherModeEnum::searchUninit;
  trackerEngine.trackerMode_  = trackerModeEnum::trackUninit;
  trackerEngine.trackerAlg = trackerAlg;
 
  switch(trackerAlg)
  {
    case BOOSTING:
      trackerEngine.tracker = cv::TrackerBoosting::create();
      break;
    case MIL:
      trackerEngine.tracker = cv::TrackerMIL::create();
       break;
    case KCF:
       trackerEngine.tracker = cv::TrackerKCF::create();
       break;
    case TLD:
       trackerEngine.tracker = cv::TrackerTLD::create();
       break;
    case MEDIANFLOW:
       trackerEngine.tracker = cv::TrackerMedianFlow::create();
       break;
    case GOTURN:
       trackerEngine.tracker = cv::TrackerGOTURN::create();
       break;
    case MOSSE:
       trackerEngine.tracker = cv::TrackerMOSSE::create();
       break;
    case CSRT:
       trackerEngine.tracker = cv::TrackerCSRT::create();
       break;
  }

  if(!foundTrackerEngine)
    trackerEngines_.push_back(trackerEngine);
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

      switch(searcherMode_)
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
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int tt_tracker::trackloop()
{ 
    double trackingFailureTimeoutTicks = cv::getTickFrequency() * trackingFailureTimeoutSeconds_;
    double timer;

    while(true)
    {     
      //wait for an image to appear
      cam_image_ready_.wait();

      //just loop (avoid locking) if we dont have work
      //if(trackerMode_ != startTracking & trackerMode_ != tracking)
      //  continue;

      ROS_DEBUG("[tt_tracker] image received.");
     
      //lock the image so new images don't overwrite
      boost::shared_lock<boost::shared_mutex> lock(mutexInputRawImage_);
      
      try
      {
        //TEST
        //cv::imshow(DisplayWindowName_, fff);
        //cv::waitKey(3);
        //continue;
      }
      catch( ... )
      {
        ROS_ERROR("exception");
        continue;
      }

      cv::Rect2d beforeBox;
      
      for( auto &trackerEngine : trackerEngines_)
      {
      switch(trackerEngine.trackerMode_)
      {
        case tracking:

          // Start timer
          timer = (double)cv::getTickCount();
          
          //trackerOk = tracker_->update(in_raw_image_->image, trackingBox_);

          // Update the tracking result
          //for( auto &trackerEngine : trackerEngines_)
          //{
            // TODO : make this work with multiple trackers

          beforeBox.x = trackerEngine.trackingBox.x;
          beforeBox.y = trackerEngine.trackingBox.y;
          beforeBox.height = trackerEngine.trackingBox.height;
          beforeBox.width = trackerEngine.trackingBox.width;


            trackerEngine.isOK = trackerEngine.tracker->update(in_raw_image_->image, trackerEngine.trackingBox);

            //todo: just fake for now
            trackerOk = trackerEngine.isOK;
          //}

          // Calculate Frames per second (FPS)
          trackPerfFps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

          if(trackerOk)
          {
              // Get a timestamp
              lastGoodTrackTickCount_ = (double)cv::getTickCount();
          }
          else
          {
            // If we have lost tack for x amount of time then re initiate search mode
            if( (cv::getTickCount() - lastGoodTrackTickCount_) > trackingFailureTimeoutTicks )
            {
              trackerEngine.trackerMode_ = trackerModeEnum::resetTracker;  
            }
          }
            
        break;

        case startTracking:

          //trackingBox_.x = trackThisBox_.x;
          //trackingBox_.y = trackThisBox_.y;
          //trackingBox_.width = trackThisBox_.width;
          //trackingBox_.height = trackThisBox_.height;
   
          //tracker_->init(in_raw_image_->image, trackingBox_);

          //for( auto &trackerEngine : trackerEngines_)
          //{
            trackerEngine.trackingBox.x = trackThisBox_.x;
            trackerEngine.trackingBox.y = trackThisBox_.y;
            trackerEngine.trackingBox.width = trackThisBox_.width;
            trackerEngine.trackingBox.height = trackThisBox_.height;

            // TODO : make this work with multiple trackers
            // TODO : allow dynamic add/remove of trackers
            trackerEngine.isOK = trackerEngine.tracker->init(in_raw_image_->image, trackerEngine.trackingBox);
            trackerEngine.trackerMode_ = trackerModeEnum::tracking;
          //}

          // Display bounding box. 
          //cv::rectangle(in_raw_image_->image, trackingBox_, cv::Scalar( 255, 0, 0 ), 2, 1 ); 
          //cv::imshow(DisplayWindowName_, in_raw_image_->image); 

          //trackerMode_ = tracking;

        break;

        case resetTracker:

          //maybe we shouldn't reinit
          addTracker(trackerEngine.trackerAlg, trackerEngine.name);
          searcherMode_ = searching;

        break;
      }
      }

      //is anybody tracking? Other threads are running, so dont use 
      //aggregateTrackingMode_ as a temp. Only change it once as needed.
      bool somebodyIsTracking = false;

      for( auto &trackerEngine : trackerEngines_)
        if(trackerEngine.trackerMode_ == trackerModeEnum::tracking)
          somebodyIsTracking = true;

      if(somebodyIsTracking)
        aggregateTrackingMode_ = trackerModeEnum::tracking;
      else
        aggregateTrackingMode_ = trackerModeEnum::trackUninit;

      //try to lock image, don't get hung up
      boost::shared_lock<boost::shared_mutex> outlock(mutexOutputImage_, boost::try_to_lock);

      // if we cant lock it then just drop this frame
      if(!outlock)
        continue;

      in_raw_image_->image.copyTo(out_image_);
      out_image_ready_.post();
    }
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

int tt_tracker::displayloop()
{ 
    cv::namedWindow(DisplayWindowName_);
    int count = 0;
    tt_tracker::releativeCoordsStruct releativeCoords;
    cv::Point objectCOM;
    cv::Point IamgeCOM(frameWidth_/2,frameHeight_/2);

    while(true)
    {     
      //wait for an image to appear
      out_image_ready_.wait();

      //just loop (avoid locking) if we dont have work
      //if(trackerMode_ != startTracking & trackerMode_ != tracking)
      //  continue;

      ROS_DEBUG("[tt_tracker] output image ready.");
    
      //lock the image so new images don't overwrite
      boost::shared_lock<boost::shared_mutex> lock(mutexOutputImage_);

      //Show searcher mode if appropriate
      switch(searcherMode_)
      {
        case searching:
          //putText(out_image_, "Searching for: " + searchForClassName_ , cv::Point(50,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);

          // Display tracker type on frame
          putText(out_image_, 
            str(boost::format("Searching for: %s") % searchForClassName_ ), 
            cv::Point(30,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
          
          // Display gimbal orientation on frame
          if(haveValidGimbalAngle_)
            putText(out_image_, 
              str(boost::format("Gimbal p: %.1f r: %.1f y: %.1f") % gimbalAngle_.vector.y % gimbalAngle_.vector.x % gimbalAngle_.vector.z), 
              cv::Point(30,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
          else
            putText(out_image_, "Gimbal : Unavailable",
              cv::Point(30,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);

        break;

        case searchUninit:
          putText(out_image_, "Not searching" , cv::Point(100,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
        break;
      }    

      /*
      try
      {
        //TEST
        cv::imshow(DisplayWindowName_, out_image_);
        cv::waitKey(1);
        continue;
      }
      catch( ... )
      {
        ROS_ERROR("exception");
        continue;
      }
      */

      objectCOM.x = 0;
      objectCOM.y = 0;
      
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
                
              objectCOM = (trackerEngine.trackingBox.br() + trackerEngine.trackingBox.tl())*0.5;
              line(out_image_, IamgeCOM, objectCOM, cv::Scalar( 0, 0, 255 ), 2, 1 );
              circle(out_image_, IamgeCOM, 10, cv::Scalar( 0, 0, 255 ), 2, 1 );

              releativeCoords = findPositionRelativeToImageCenter(trackerEngine.trackingBox, frameHeight_, frameWidth_ );

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
              TrackBoxesPublisher_.publish(TrackBoxesResults_);
            }
            else
            {
              // Tracking failure detected.
              putText(out_image_, "Tracking failure detected", 
                cv::Point(100,80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255),2);
            }
          }

          // Display tracker type on frame
          putText(out_image_, 
            str(boost::format("%s : %s(%s) %.2f%% : %s") 
              % trackerType_ % currentTrackModeMessage_.modeName % searchForClassName_ 
              % foundObjcetClassProbability % SSTR(int(trackPerfFps))), 
            cv::Point(30,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
          
          // Display offset on frame
          putText(out_image_, 
            str(boost::format("w: %.3f h: %.3f") % releativeCoords.wPercent % releativeCoords.hPercent), 
            cv::Point(30,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);

          // Display gimbal orientation on frame
          if(haveValidGimbalAngle_)
          putText(out_image_, 
            str(boost::format("Gimbal p: %.1f r: %.1f y: %.1f") 
              % gimbalAngle_.vector.y % gimbalAngle_.vector.x % gimbalAngle_.vector.z), 
            cv::Point(30,80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
          else
            putText(out_image_, "Gimbal : Unavailable",
              cv::Point(30,80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
           
        break;
      }

      //***********************************************

      /*std_msgs::String msg;

      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());

      chatter_pub_.publish(msg);

      ++count;*/

      //***********************************************

      // Display frame.
      cv::imshow(DisplayWindowName_, out_image_);
          
      // Exit if ESC pressed.
      int k = cv::waitKey(1);
      
      if(k == 27)
      {
        break;
      }
    }
}
}
