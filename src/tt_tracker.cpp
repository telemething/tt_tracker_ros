/*
 * tt_tracker_ros_node.cpp
 *
 *  Created on: 1.1.1
 *      Author: Markn West
 *   
 */

 // yolo object detector
#include "tt_tracker_ros/tt_tracker.hpp"

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

  startSearchingForObject("bottle");
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

  initTracker();

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  std::string darknetBoundingBoxesName;
  int cameraQueueSize;  
  int darknetBoundingBoxesQueueSize;

  // Read Config
  nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
                    std::string("/camera/image_raw"));
                    
  nodeHandle_.param("darknet_ros/bounding_boxes/topic", darknetBoundingBoxesName,
                    std::string("/darknet_ros/bounding_boxes"));
                    
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);  
  nodeHandle_.param("darknet_ros/bounding_boxes/queue_size", darknetBoundingBoxesQueueSize, 1);

  //cv::namedWindow(DisplayWindowName_);

  // Subscribe
  imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize,
                                               &tt_tracker::cameraCallback, this);

  darknetBoundingBoxesSubscriber_ = nodeHandle_.subscribe(darknetBoundingBoxesName, darknetBoundingBoxesQueueSize,
                                               &tt_tracker::darknetBoundingBoxesCallback, this);

  // Start threads                                             
  trackloop_thread = std::thread(&tt_tracker::trackloop, this); 
  //searchloop_thread = std::thread(&tt_tracker::searchloop, this); 
  displayloop_thread = std::thread(&tt_tracker::displayloop, this);  
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void tt_tracker::startSearchingForObject(const std::string className)
{
  trackerMode_ = stopTracking;
  searchForClassName_ = className;
  searcherMode_ = searching;
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
        trackerMode_ = startTracking;
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
//******************************************************************************

int tt_tracker::initTracker()
{
    // List of tracker types in OpenCV 3.4.1
    std::string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    // vector <string> trackerTypes(types, std::end(types));

    if(NULL != tracker_)
      tracker_.release();
 
    // Create a tracker
    trackerType_ = trackerTypes[2];
 
    #if (CV_MINOR_VERSION < 3)
    {
        tracker_ = Tracker::create(trackerType_);
    }
    #else
    {
        if (trackerType_ == "BOOSTING")
            tracker_ = cv::TrackerBoosting::create();
        if (trackerType_ == "MIL")
            tracker_ = cv::TrackerMIL::create();
        if (trackerType_ == "KCF")
            tracker_ = cv::TrackerKCF::create();
        if (trackerType_ == "TLD")
            tracker_ = cv::TrackerTLD::create();
        if (trackerType_ == "MEDIANFLOW")
            tracker_ = cv::TrackerMedianFlow::create();
        if (trackerType_ == "GOTURN")
            tracker_ = cv::TrackerGOTURN::create();
        if (trackerType_ == "MOSSE")
            tracker_ = cv::TrackerMOSSE::create();
        if (trackerType_ == "CSRT")
            tracker_ = cv::TrackerCSRT::create();
    }
    #endif
}


//*****************************************************************************
//*
//*
//*
//******************************************************************************

int tt_tracker::trackloop()
{ 
    double trackingFailureTimeoutTicks = cv::getTickFrequency() * trackingFailureTimeoutSeconds;
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
      
      switch(trackerMode_)
      {
        case tracking:

          // Start timer
          timer = (double)cv::getTickCount();
          
          // Update the tracking result
          trackerOk = tracker_->update(in_raw_image_->image, trackingBox_);
          
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
              trackerMode_ = resetTracker;  
            }
          }
            
        break;

        case startTracking:

          trackingBox_.x = trackThisBox_.x;
          trackingBox_.y = trackThisBox_.y;
          trackingBox_.width = trackThisBox_.width;
          trackingBox_.height = trackThisBox_.height;
   
          tracker_->init(in_raw_image_->image, trackingBox_);

          // Display bounding box. 
          //cv::rectangle(in_raw_image_->image, trackingBox_, cv::Scalar( 255, 0, 0 ), 2, 1 ); 
          //cv::imshow(DisplayWindowName_, in_raw_image_->image); 

          trackerMode_ = tracking;

        break;

        case resetTracker:

          initTracker();
          searcherMode_ = searching;

        break;
      }

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

int tt_tracker::displayloop()
{ 
    cv::namedWindow(DisplayWindowName_);


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
          putText(out_image_, "Searching for: " + searchForClassName_ , cv::Point(100,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
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
      
      switch(trackerMode_)
      {
        case startTracking:

        break;

        case tracking:
         
          if (trackerOk)
          {
              // Tracking success : Draw the tracked object
              rectangle(out_image_, trackingBox_, cv::Scalar( 255, 0, 0 ), 2, 1 );
          }
          else
          {
              // Tracking failure detected.
              putText(out_image_, "Tracking failure detected", cv::Point(100,80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255),2);
          }
          
          // Display tracker type on frame
          putText(out_image_, trackerType_ + " Tracker", cv::Point(100,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
          
          // Display FPS on frame
          putText(out_image_, "FPS : " + SSTR(int(trackPerfFps)), cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50), 2);
  
        break;
      }

      // Display frame.
      cv::imshow("Tracking", out_image_);
          
      // Exit if ESC pressed.
      int k = cv::waitKey(1);
      
      if(k == 27)
      {
        break;
      }
    }
}


//*****************************************************************************
//*
//*
//*
//******************************************************************************

/*int tt_tracker::searchloop()
{    
    while(true)
    {     
      //cam_image_ready_.wait();
      //cv::Mat frame = cam_image_->image;

      switch(searcherMode_)
      {
        case startSearching:

        break;

        case searching:

          //when found
          trackThisBox_.x = 0;
          trackThisBox_.y = 0;
          trackThisBox_.width = 0;
          trackThisBox_.height = 0;

          searcherMode_ = found;
          trackerMode_ = startTracking;

        break;
      }
    }
}*/

}
