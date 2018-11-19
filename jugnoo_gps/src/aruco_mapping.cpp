/*********************************************************************************************//**
* @file aruco_mapping.cpp
*
* Copyright (c)
* Smart Robotic Systems
* March 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/* Author: Jan Bacik */

#ifndef ARUCO_MAPPING_CPP
#define ARUCO_MAPPING_CPP

#include <aruco_mapping.h>

namespace aruco_mapping
{

ArucoMapping::ArucoMapping(ros::NodeHandle *nh) :
  listener_ (new tf::TransformListener),  // Initialize TF Listener  
  num_of_markers_ (10),                   // Number of used markers
  marker_size_(0.1),                      // Marker size in m
  calib_filename_("empty"),               // Calibration filepath
  space_type_ ("plane"),                  // Space type - 2D plane 
  roi_allowed_ (false),                   // ROI not allowed by default
  first_marker_detected_(false),          // First marker not detected by defualt
  lowest_marker_id_(-1),                  // Lowest marker ID
  marker_counter_(0),                     // Reset marker counter
  closest_camera_index_(0)                // Reset closest camera index 
  
{
  double temp_marker_size;  
  
  //Parse params from launch file 
  nh->getParam("/aruco_mapping/calibration_file", calib_filename_);
  nh->getParam("/aruco_mapping/marker_size", temp_marker_size); 
  nh->getParam("/aruco_mapping/num_of_markers", num_of_markers_);
  nh->getParam("/aruco_maping/space_type",space_type_);
  nh->getParam("/aruco_mapping/roi_allowed",roi_allowed_);
  nh->getParam("/aruco_mapping/roi_x",roi_x_);
  nh->getParam("/aruco_mapping/roi_y",roi_y_);
  nh->getParam("/aruco_mapping/roi_w",roi_w_);
  nh->getParam("/aruco_mapping/roi_h",roi_h_);
     
  // Double to float conversion
  marker_size_ = float(temp_marker_size);
  
    
  //ROS publishers
  camera_pub_ = nh->advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1);
          
  //Initialize OpenCV window
  cv::namedWindow("Mono8", CV_WINDOW_AUTOSIZE);       
      
  //Resize marker container
  markers_.resize(num_of_markers_);
  
  // Default markers_ initialization
  for(size_t i = 0; i < num_of_markers_;i++)
  {
    markers_[i].previous_marker_id = -1;
    markers_[i].visible = false;
    markers_[i].marker_id = -1;
  }
}

ArucoMapping::~ArucoMapping()
{
 delete listener_;
}

void
ArucoMapping::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  }
  
  // sensor_msgs::Image to OpenCV Mat structure
  cv::Mat I = cv_ptr->image;
  
  // region of interest
  if(roi_allowed_==true)
    I = cv_ptr->image(cv::Rect(roi_x_,roi_y_,roi_w_,roi_h_));

  //Marker detection
  processImage(I,I);
  
  // Show image
  cv::imshow("Mono8", I);
  cv::waitKey(10);  
}

void ArucoMapping::imageReader()
{
    cv::VideoCapture in_video;
    in_video.open(0);
    cv::Mat image, outp;

    while (in_video.grab())
    {
        in_video.retrieve(image);
        processImage(image, outp);
        char key = (char) cv::waitKey(1);
        if (key == 27)
            break;
    }
    
    in_video.release();

}

bool
ArucoMapping::processImage(cv::Mat input_image,cv::Mat output_image)
{
  Aruco a;
  geometry_msgs::Pose pose ;
  a.init();
  a.detect(input_image, world_position_geometry_msg_);
  publishCamera(world_position_geometry_msg_);

}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishCamera(geometry_msgs::Pose marker_pose)
{
  visualization_msgs::Marker vis_marker;

  geometry_msgs::PoseStamped msg_pose;

  msg_pose.header.stamp = ros::Time::now();

  msg_pose.header.frame_id = "map";

  msg_pose.pose = marker_pose;

  camera_pub_.publish(msg_pose);
}




}  //aruco_mapping

#endif  //ARUCO_MAPPING_CPP
