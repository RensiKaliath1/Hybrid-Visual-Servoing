#ifndef MAIN_ROS_H_
#define MAIN_ROS_H_

// C++
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
// ROS
#include "ros/package.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
// Own libraries
#include <filesystem/filesystem.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/texture_modality.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/manual_detector.h>
#include <m3t/normal_viewer.h>
#include <m3t/publisher.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/ros_camera.h>
#include <m3t/ros_publisher.h>
#include <m3t/static_detector.h>
#include <m3t/tracker.h>

class Rosm3tTracker
{
public:
    Rosm3tTracker (ros::NodeHandle *nodehandle);

private:
    int initm3t();
    int runm3t();

    // Callbacks
    void rgbImageCallback (const sensor_msgs::ImageConstPtr &msg);
    void depthImageCallback (const sensor_msgs::ImageConstPtr &msg);
    void rgbCameraInfoCallback (const sensor_msgs::CameraInfoConstPtr cameraInfo);
    void depthCameraInfoCallback (const sensor_msgs::CameraInfoConstPtr cameraInfo);
    void ShutdownNode(const std_msgs::Bool shutdown);
    // Subscribers
    image_transport::Subscriber imageSubRgb;
    image_transport::Subscriber imageSubDepth;
    ros::Subscriber rgbCameraInfoSub;
    ros::Subscriber depthCameraInfoSub;
    ros::Subscriber icgShutdownSubscriber;
    // Publishers
    image_transport::Publisher pose_estimator_pub;
    std::shared_ptr<m3t::RosPublisher> publisher_ptr;
    ros::Publisher posePub;
    std::string cameraTopic;
    std::string initial_approach;

    // Internal state variables
    bool firstInitRgb = false;
    bool firstInitDepth = false;
    bool firstImageRgb = false;
    bool firstImageDepth = false;
    bool initDone = false;
    bool runm3tDone = false;
    bool useDepthCamera = false;
    ros::NodeHandle *nh;
    std::string servo_type;
    std::string task;
    std::string manualDetectorFilename;
    std::vector<std::string> body_names_;
    cv_bridge::CvImagePtr cv_ptr_rgb;
    cv_bridge::CvImagePtr cv_ptr_depth;

    std::shared_ptr<m3t::RosCameraColorCamera> color_camera_ptr;
    std::shared_ptr<m3t::RosCameraDepthCamera> depth_camera_ptr;
    std::shared_ptr<m3t::Tracker> tracker_ptr;
    std::thread m3tThread;
};

#endif
