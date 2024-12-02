// Partially written by Object Handling Group

#include <m3t/ros_camera.h>
#include <mutex>

namespace m3t
{

    RosCamera &RosCamera::GetInstance()
    {
        static RosCamera roscamera;
        return roscamera;
    }

    RosCamera::~RosCamera()
    {
        if (initial_set_up_)
        {
            // device_.stop_cameras();
            // device_.close();
        }
    }

    void RosCamera::UseColorCamera() { use_color_camera_ = true; }

    void RosCamera::UseDepthCamera() { use_depth_camera_ = true; }

    int RosCamera::RegisterID()
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        update_capture_ids_.insert (std::pair<int, bool>{next_id_, true});
        return next_id_++;
    }

    bool RosCamera::UnregisterID (int id)
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        return update_capture_ids_.erase (id);
    }

    bool RosCamera::SetUp()
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        if (!initial_set_up_)
        {
            // Get extrinsics and calculate pose
            if (use_color_camera_ && use_depth_camera_)
            {
                // TODO: Extrinsics are currently hard coded
                float rotation[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
                Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot{rotation};
                float translation[3] = {-25.0f, 0.0f, 0.0f};
                Eigen::Vector3f trans{translation};
                color2depth_pose_.setIdentity();
                color2depth_pose_.translate (trans * 0.001f);
                color2depth_pose_.rotate (rot);
                depth2color_pose_ = color2depth_pose_.inverse();
            }
            initial_set_up_ = true;
        }
        return true;
    }

    bool RosCameraColorCamera::UpdateCapture (cv::Mat cv_img)
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        new_image_ = true;
        capture_rgb_ = cv_img;
        cv_.notify_one();

        return true;
    }

    bool RosCameraDepthCamera::UpdateCapture (cv::Mat cv_img)
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        new_image_ = true;
        capture_depth_ = cv_img;
        cv_.notify_one();
        return true;
    }

    bool RosCamera::use_color_camera() const { return use_color_camera_; }

    bool RosCamera::use_depth_camera() const { return use_depth_camera_; }

    const Transform3fA *RosCamera::color2depth_pose() const
    {
        if (initial_set_up_)
            return &color2depth_pose_;
        else
            return nullptr;
    }

    const Transform3fA *RosCamera::depth2color_pose() const
    {
        if (initial_set_up_)
            return &depth2color_pose_;
        else
            return nullptr;
    }

    RosCameraColorCamera::RosCameraColorCamera (const std::string &name,
                                                const sensor_msgs::CameraInfoConstPtr &camera_info,
                                                const std::filesystem::path &metafile_path,
                                                float image_scale,
                                                bool use_depth_as_world_frame)
        : ColorCamera{name},
          camera_info_{camera_info},
          metafile_path_{metafile_path},
          image_scale_{image_scale},
          use_depth_as_world_frame_{use_depth_as_world_frame},
          ros_camera_{RosCamera::GetInstance()}
    {
        ros_camera_.UseColorCamera();
        ros_camera_id_ = ros_camera_.RegisterID();
    }

    RosCameraColorCamera::RosCameraColorCamera (
        const std::string &name, const sensor_msgs::CameraInfoConstPtr &camera_info)
        : ColorCamera{name}, camera_info_{camera_info},
          ros_camera_{RosCamera::GetInstance()}
    {
        ros_camera_.UseColorCamera();
        ros_camera_id_ = ros_camera_.RegisterID();
    }

    RosCameraColorCamera::~RosCameraColorCamera()
    {
        ros_camera_.UnregisterID (ros_camera_id_);
    }

    bool RosCameraColorCamera::SetUp()
    {
        set_up_ = false;
        if (!metafile_path_.empty())
            if (!LoadMetaData())
                return false;
        if (!initial_set_up_ && !ros_camera_.SetUp())
            return false;

        if (use_depth_as_world_frame_)
            set_camera2world_pose (*ros_camera_.color2depth_pose());
            
        GetIntrinsicsAndDistortionMap();
        SaveMetaDataIfDesired();
        set_up_ = true;
        initial_set_up_ = true;
        return UpdateImage (true);
        //return true;
    }

    void RosCameraColorCamera::set_image_scale (float image_scale)
    {
        image_scale_ = image_scale;
        set_up_ = false;
    }

    void RosCameraColorCamera::set_use_depth_as_world_frame (
        bool use_depth_as_world_frame)
    {
        use_depth_as_world_frame_ = use_depth_as_world_frame;
        set_up_ = false;
    }

    bool RosCameraColorCamera::UpdateImage (bool synchronized)
    {
        if (!set_up_)
        {
            std::cerr << "Set up ros color camera " << name_ << " first"
                      << std::endl;
            return false;
        }
        // Wait until new image arrives
        std::unique_lock lk(mutex_);
        cv_.wait(lk, [this]{return new_image_;});
        new_image_ = false;
        
        capture_rgb_.copyTo (image_);
        SaveImageIfDesired();
        return true;
    }

    float RosCameraColorCamera::image_scale() const { return image_scale_; }

    sensor_msgs::CameraInfoConstPtr RosCameraColorCamera::camera_info() const { return camera_info_; }

    bool RosCameraColorCamera::use_depth_as_world_frame() const
    {
        return use_depth_as_world_frame_;
    }

    const Transform3fA *RosCameraColorCamera::color2depth_pose() const
    {
        return ros_camera_.color2depth_pose();
    }

    const Transform3fA *RosCameraColorCamera::depth2color_pose() const
    {
        return ros_camera_.depth2color_pose();
    }

    bool RosCameraColorCamera::LoadMetaData()
    {
        // Open file storage from yaml
        cv::FileStorage fs;
        if (!OpenYamlFileStorage (metafile_path_, &fs))
            return false;

        // Read parameters from yaml
        ReadOptionalValueFromYaml (fs, "camera2world_pose", &camera2world_pose_);
        ReadOptionalValueFromYaml (fs, "save_directory", &save_directory_);
        ReadOptionalValueFromYaml (fs, "save_index", &save_index_);
        ReadOptionalValueFromYaml (fs, "save_image_type", &save_image_type_);
        ReadOptionalValueFromYaml (fs, "save_images", &save_images_);
        ReadOptionalValueFromYaml (fs, "image_scale", &image_scale_);
        ReadOptionalValueFromYaml (fs, "use_depth_as_world_frame",
                                   &use_depth_as_world_frame_);
        fs.release();

        // Process parameters
        if (save_directory_.is_relative())
            save_directory_ = metafile_path_.parent_path() / save_directory_;
        world2camera_pose_ = camera2world_pose_.inverse();
        return true;
    }

    void RosCameraColorCamera::GetIntrinsicsAndDistortionMap()
    {
        // Load intrinsics from camera
        double fx = camera_info_->K[0];
        double fy = camera_info_->K[4];
        double cx = camera_info_->K[2];
        double cy = camera_info_->K[5];
        intrinsics_.fu = fx;
        intrinsics_.fv = fy;
        intrinsics_.ppu = cx;
        intrinsics_.ppv = cy;
        intrinsics_.width = camera_info_->width;
        intrinsics_.height = camera_info_->height;

        // Scale intrinsics according to image scale
        intrinsics_.fu *= image_scale_;
        intrinsics_.fv *= image_scale_;

        // Calculate distortion map
        cv::Mat1f camera_matrix (3, 3);
        camera_matrix << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        cv::Mat1f new_camera_matrix (3, 3);
        new_camera_matrix << intrinsics_.fu, 0, intrinsics_.ppu, 0, intrinsics_.fv,
            intrinsics_.ppv, 0, 0, 1;
        cv::Mat1f distortion_coeff (1, 5);
        distortion_coeff << camera_info_->D[0], camera_info_->D[1], camera_info_->D[2],
            camera_info_->D[3], camera_info_->D[4];
        cv::Mat map1, map2, map3;
        cv::initUndistortRectifyMap (
            camera_matrix, distortion_coeff, cv::Mat{}, new_camera_matrix,
            cv::Size{intrinsics_.width, intrinsics_.height}, CV_32FC1, map1, map2);
        cv::convertMaps (map1, map2, distortion_map_, map3, CV_16SC2, true);
    }

    RosCameraDepthCamera::RosCameraDepthCamera (const std::string &name,
                                                const sensor_msgs::CameraInfoConstPtr &camera_info,
                                                float image_scale,
                                                bool use_color_as_world_frame)
        : DepthCamera{name},
          camera_info_{camera_info},
          image_scale_{image_scale},
          use_color_as_world_frame_{use_color_as_world_frame},
          ros_camera_{RosCamera::GetInstance()}
    {
        ros_camera_.UseDepthCamera();
        ros_camera_id_ = ros_camera_.RegisterID();
    }

    RosCameraDepthCamera::RosCameraDepthCamera (
        const std::string &name, const std::filesystem::path &metafile_path)
        : DepthCamera{name, metafile_path},
          ros_camera_{RosCamera::GetInstance()}
    {
        ros_camera_.UseDepthCamera();
        ros_camera_id_ = ros_camera_.RegisterID();
    }

    RosCameraDepthCamera::~RosCameraDepthCamera()
    {
        ros_camera_.UnregisterID (ros_camera_id_);
    }

    bool RosCameraDepthCamera::SetUp()
    {
        set_up_ = false;
        if (!metafile_path_.empty())
            if (!LoadMetaData())
                return false;
        if (!initial_set_up_ && !ros_camera_.SetUp())
            return false;
        if (use_color_as_world_frame_)
            set_camera2world_pose (*ros_camera_.depth2color_pose());
        GetIntrinsicsAndDistortionMap();
        SaveMetaDataIfDesired();
        set_up_ = true;
        initial_set_up_ = true;
        return UpdateImage (true);
    }

    void RosCameraDepthCamera::set_image_scale (float image_scale)
    {
        image_scale_ = image_scale;
        set_up_ = false;
    }

    void RosCameraDepthCamera::set_use_color_as_world_frame (
        bool use_color_as_world_frame)
    {
        use_color_as_world_frame_ = use_color_as_world_frame;
        set_up_ = false;
    }

    bool RosCameraDepthCamera::UpdateImage (bool synchronized)
    {
        if (!set_up_)
        {
            std::cerr << "Set up ros depth camera " << name_ << " first"
                      << std::endl;
            return false;
        }
        if (new_image_)
        {
            new_image_ = false;
        }
        else
        {
            // Wait until new image arrives
            std::unique_lock lk(mutex_);
            cv_.wait(lk, [this]{return new_image_;});
        }
        capture_depth_.copyTo (image_);

        SaveImageIfDesired();
        return true;
    }

    float RosCameraDepthCamera::image_scale() const { return image_scale_; }

    sensor_msgs::CameraInfoConstPtr RosCameraDepthCamera::camera_info() const { return camera_info_; }

    bool RosCameraDepthCamera::use_color_as_world_frame() const
    {
        return use_color_as_world_frame_;
    }

    const Transform3fA *RosCameraDepthCamera::color2depth_pose() const
    {
        return ros_camera_.color2depth_pose();
    }

    const Transform3fA *RosCameraDepthCamera::depth2color_pose() const
    {
        return ros_camera_.depth2color_pose();
    }

    bool RosCameraDepthCamera::LoadMetaData()
    {
        // Open file storage from yaml
        cv::FileStorage fs;
        if (!OpenYamlFileStorage (metafile_path_, &fs))
            return false;

        // Read parameters from yaml
        ReadOptionalValueFromYaml (fs, "camera2world_pose", &camera2world_pose_);
        ReadOptionalValueFromYaml (fs, "save_directory", &save_directory_);
        ReadOptionalValueFromYaml (fs, "save_index", &save_index_);
        ReadOptionalValueFromYaml (fs, "save_image_type", &save_image_type_);
        ReadOptionalValueFromYaml (fs, "save_images", &save_images_);
        ReadOptionalValueFromYaml (fs, "image_scale", &image_scale_);
        ReadOptionalValueFromYaml (fs, "use_color_as_world_frame",
                                   &use_color_as_world_frame_);
        fs.release();

        // Process parameters
        if (save_directory_.is_relative())
            save_directory_ = metafile_path_.parent_path() / save_directory_;
        world2camera_pose_ = camera2world_pose_.inverse();
        return true;
    }

    void RosCameraDepthCamera::GetIntrinsicsAndDistortionMap()
    {
        // Load intrinsics from camera
        double fx = camera_info_->K[0];
        double fy = camera_info_->K[4];
        double cx = camera_info_->K[2];
        double cy = camera_info_->K[5];
        intrinsics_.fu = fx;
        intrinsics_.fv = fy;
        intrinsics_.ppu = cx;
        intrinsics_.ppv = cy;
        intrinsics_.width = camera_info_->width;
        intrinsics_.height = camera_info_->height;
        depth_scale_ = 0.001f;

        // Scale intrinsics according to image scale
        intrinsics_.fu *= image_scale_;
        intrinsics_.fv *= image_scale_;

        // Calculate distortion map
        cv::Mat1f camera_matrix (3, 3);
        camera_matrix << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        cv::Mat1f new_camera_matrix (3, 3);
        new_camera_matrix << intrinsics_.fu, 0, intrinsics_.ppu, 0, intrinsics_.fv,
            intrinsics_.ppv, 0, 0, 1;
        cv::Mat1f distortion_coeff (1, 5);
        distortion_coeff << camera_info_->D[0], camera_info_->D[1], camera_info_->D[2],
            camera_info_->D[3], camera_info_->D[4];
        cv::Mat map1, map2, map3;
        cv::initUndistortRectifyMap (
            camera_matrix, distortion_coeff, cv::Mat{}, new_camera_matrix,
            cv::Size{intrinsics_.width, intrinsics_.height}, CV_32FC1, map1, map2);
        cv::convertMaps (map1, map2, distortion_map_, map3, CV_16SC2, true);
    }

} // namespace m3t
