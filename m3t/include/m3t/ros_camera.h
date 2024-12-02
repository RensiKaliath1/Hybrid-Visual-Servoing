#ifndef m3t_INCLUDE_m3t_ROS_CAMERA_H_
#define m3t_INCLUDE_m3t_ROS_CAMERA_H_

#include <filesystem/filesystem.h>
#include <m3t/camera.h>
#include <m3t/common.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <condition_variable>

// ROS
#include "ros/package.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

namespace m3t
{

    /**
     * \brief Singleton class that allows getting data from a single Azure Kinect
     * instance and that is used by \ref RosCameraColorCamera and \ref
     * RosCameraDepthCamera.
     *
     * \details The method `UpdateCapture()` updates the `capture` object if
     * `UpdateCapture()` was already called with the same `id` before. If
     * `UpdateCapture()` was not yet called by the `id`, the same capture is used.
     * If the capture is updated, all memory values except for the `id` that called
     * the function are reset. All methods that are required to operate multiple
     * \ref Camera objects are thread-safe.
     */
    class RosCamera
    {
    public:
        // Singleton instance getter
        static RosCamera &GetInstance();
        RosCamera (const RosCamera &) = delete;
        void operator= (const RosCamera &) = delete;
        ~RosCamera();

        // Configuration and setup
        void UseColorCamera();
        void UseDepthCamera();
        int RegisterID();
        bool UnregisterID (int id);
        bool SetUp();

        // Main methods
        // bool UpdateCapture(cv_bridge::CvImagePtr cv_ptr);

        // Getters
        bool use_color_camera() const;
        bool use_depth_camera() const;
        // const k4a::capture &capture() const;
        // const k4a::calibration &calibration() const;
        const Transform3fA *color2depth_pose() const;
        const Transform3fA *depth2color_pose() const;

    private:
        RosCamera() = default;

        // Private data
        // k4a::device device_{};
        // k4a_device_configuration_t config_{};
        std::map<int, bool> update_capture_ids_{};
        int next_id_ = 0;

        // Public data
        // k4a::capture capture_{};
        // k4a::calibration calibration_{};
        Transform3fA color2depth_pose_{Transform3fA::Identity()};
        Transform3fA depth2color_pose_{Transform3fA::Identity()};

        // Internal state variables
        std::mutex mutex_;
        bool use_color_camera_ = false;
        bool use_depth_camera_ = false;
        bool initial_set_up_ = false;
    };

    /**
     * \brief \ref Camera that allows getting color images from an \ref RosCamera
     * camera.
     *
     * @param image_scale scales images to avoid borders after rectification.
     * @param use_depth_as_world_frame specifies the depth camera frame as world
     * frame and automatically defines `camera2world_pose` as `color2depth_pose`.
     */
    class RosCameraColorCamera : public ColorCamera
    {
    public:
        // Constructors, destructor, and setup method
        RosCameraColorCamera (const std::string &name,
                              const sensor_msgs::CameraInfoConstPtr &camera_info,
                              const std::filesystem::path &metafile_path,
                              float image_scale = 1.0f,
                              bool use_depth_as_world_frame = false);
        RosCameraColorCamera (const std::string &name, const sensor_msgs::CameraInfoConstPtr &camera_info);
        ~RosCameraColorCamera();
        bool SetUp() override;

        // Setters
        void set_image_scale (float image_scale);
        void set_use_depth_as_world_frame (bool use_depth_as_world_frame);

        // Main method
        bool UpdateImage (bool synchronized) override;
        bool UpdateCapture (cv::Mat cv_img);

        // Getters
        float image_scale() const;
        bool use_depth_as_world_frame() const;
        const Transform3fA *color2depth_pose() const;
        const Transform3fA *depth2color_pose() const;
        sensor_msgs::CameraInfoConstPtr camera_info() const;

        // cv_bridge::CvImagePtr capture_rgb_;
        cv::Mat capture_rgb_;

    private:
        // Helper methods
        bool LoadMetaData();
        void GetIntrinsicsAndDistortionMap();

        // Data
        int ros_camera_id_{};
        sensor_msgs::CameraInfoConstPtr camera_info_{};
        std::filesystem::path metafile_path_{};
        Transform3fA camera2world_pose_{Transform3fA::Identity()};
        Transform3fA world2camera_pose_{Transform3fA::Identity()};
        float image_scale_ = 1.0f;
        bool use_depth_as_world_frame_ = false;
        RosCamera &ros_camera_;
        cv::Mat distortion_map_;
        bool initial_set_up_ = false;
        bool new_image_ = false;
        std::condition_variable cv_;

        std::mutex mutex_;
    };

    /**
     * \brief \ref Camera that allows getting color images from an \ref RosCamera
     * camera.
     *
     * @param image_scale scales image to avoid borders after rectification.
     * @param use_color_as_world_frame specifies the color camera frame as world
     * frame and automatically define `camera2world_pose` as `depth2color_pose`.
     */
    class RosCameraDepthCamera : public DepthCamera
    {
    public:
        // Constructors, destructor, and setup method
        RosCameraDepthCamera (const std::string &name,
                              const sensor_msgs::CameraInfoConstPtr &camera_info,
                              float image_scale = 1.0f,
                              bool use_color_as_world_frame = true);
        RosCameraDepthCamera (const std::string &name,
                              const std::filesystem::path &metafile_path);
        ~RosCameraDepthCamera();
        bool SetUp() override;

        // Setters
        void set_image_scale (float image_scale);
        void set_use_color_as_world_frame (bool use_color_as_world_frame);

        // Main method
        bool UpdateImage (bool synchronized) override;
        bool UpdateCapture (cv::Mat cv_img);

        // Getters
        float image_scale() const;
        bool use_color_as_world_frame() const;
        const Transform3fA *color2depth_pose() const;
        const Transform3fA *depth2color_pose() const;
        sensor_msgs::CameraInfoConstPtr camera_info() const;

        // cv_bridge::CvImagePtr capture_depth_;
        cv::Mat capture_depth_;
        bool new_image_ = false;
    private:
        // Helper methods
        bool LoadMetaData();
        void GetIntrinsicsAndDistortionMap();

        // Data
        sensor_msgs::CameraInfoConstPtr camera_info_{};
        float image_scale_ = 1.0f;
        bool use_color_as_world_frame_ = true;
        RosCamera &ros_camera_;
        int ros_camera_id_{};
        cv::Mat distortion_map_;
        bool initial_set_up_ = false;
        std::mutex mutex_;
        std::condition_variable cv_;


    };

} // namespace m3t

#endif // m3t_INCLUDE_m3t_ROS_CAMERA_H_
