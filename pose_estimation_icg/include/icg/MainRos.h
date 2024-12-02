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
#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/common.h>
#include <icg/depth_modality.h>
#include <icg/depth_model.h>
#include <icg/manual_detector.h>
#include <icg/normal_viewer.h>
#include <icg/publisher.h>
#include <icg/region_modality.h>
#include <icg/region_model.h>
#include <icg/renderer_geometry.h>
#include <icg/ros_camera.h>
#include <icg/ros_publisher.h>
#include <icg/static_detector.h>
#include <icg/tracker.h>

class RosICGTracker
{
public:
    RosICGTracker (ros::NodeHandle *nodehandle);

private:
    int initICG();
    int runICG();

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
    std::shared_ptr<icg::RosPublisher> publisher_ptr;
    ros::Publisher posePub;
    std::string cameraTopic;
    std::string initial_approach;
    // Internal state variables
    bool firstInitRgb = false;
    bool firstInitDepth = false;
    bool firstImageRgb = false;
    bool firstImageDepth = false;
    bool initDone = false;
    bool runICGDone = false;
    bool useDepthCamera = false;
    std::string servo_type;
    ros::NodeHandle *nh;
    std::string manualDetectorFilename;
    std::string task;
    std::vector<std::string> body_names_;
    cv_bridge::CvImagePtr cv_ptr_rgb;
    cv_bridge::CvImagePtr cv_ptr_depth;

    std::shared_ptr<icg::RosCameraColorCamera> color_camera_ptr;
    std::shared_ptr<icg::RosCameraDepthCamera> depth_camera_ptr;
    std::shared_ptr<icg::Tracker> tracker_ptr;
    std::thread icgThread;
};

#endif
