#include <m3t/ros_publisher.h>

namespace m3t
{
  void RosPublisher::setParameters(ros::NodeHandle& nh)
    {
    std::string tf_prefix_param;
    std::string camera_sensor_frame_id_param;
    nh.getParam("/Dexterity/hardware_parameters/tf_prefix", tf_prefix_param);
    tf_prefix = tf_prefix_param; 
    nh.getParam("/Dexterity/hardware_parameters/camera_sensor_frame_id", camera_sensor_frame_id_param);
    cameraSensorFrame = camera_sensor_frame_id_param;
    }

    geometry_msgs::PoseStamped RosPublisher::calculateTransformStampPosition (const geometry_msgs::PoseStamped &geoPoseStampedIn, const std::string targetFrame = "kinova_gen3_base_link")
    {
        geometry_msgs::PoseStamped geoPoseStampedOut;
        try
        {
            tfListener.transformPose (targetFrame, geoPoseStampedIn, geoPoseStampedOut);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN ("%s", ex.what());
            ros::Duration (1.0).sleep();
        }
        return geoPoseStampedOut;
    }
    void RosPublisher::set_name (const std::string &name) { name_ = name; }

    void RosPublisher::set_metafile_path (const std::filesystem::path &metafile_path)
    {
        metafile_path_ = metafile_path;
        set_up_ = false;
    }

    const std::string &RosPublisher::name() const { return name_; }

    const std::filesystem::path &RosPublisher::metafile_path() const
    {
        return metafile_path_;
    }

    bool RosPublisher::set_up() const { return set_up_; }

    bool RosPublisher::SetUp()
    {
        if (!camera_ptr_)
        {
            std::cout << "Publisher class requires camera pointer!" << std::endl;
            return false;
        }
        if (!posePub)
        {
            std::cout << "Publisher class requires ros publisher pointer!" << std::endl;
            return false;
        }
        set_up_ = true;
        return true;
    }

    bool RosPublisher::AddReferencedBody (
        const std::shared_ptr<Link> &referenced_body_ptr)
    {
        //set_up_ = false;
        if (!AddPtrIfNameNotExists (referenced_body_ptr, &referenced_body_ptrs_))
        {
            std::cerr << "Referenced body " << referenced_body_ptr->name()
                      << " already exists" << std::endl;
            return false;
        }
        return true;
    }

    bool RosPublisher::DeleteReferencedBody (const std::string &name)
    {
        set_up_ = false;
        if (!DeletePtrIfNameExists (name, &referenced_body_ptrs_))
        {
            std::cerr << "Referenced body " << name << " not found" << std::endl;
            return false;
        }
        return true;
    }

    void RosPublisher::ClearReferencedBodies()
    {
        set_up_ = false;
        referenced_body_ptrs_.clear();
    }

    void RosPublisher::AddRosPublisher (ros::Publisher &pose_pub)
    {
        posePub = &pose_pub;
    }

    void RosPublisher::AddCamera (const std::shared_ptr<Camera> &camera_ptr)
    {
        camera_ptr_ = camera_ptr;
    }

    bool RosPublisher::UpdatePublisher (int iteration)
    {
        // For each tracked body, get its body2world pose and the cameras
        // world2camera pose to publish a body2camera pose for each object in
        // one message.
        for (auto &referenced_body_ptr : referenced_body_ptrs_)
        {
            auto body2camera_pose = camera_ptr_->world2camera_pose() *
                                    referenced_body_ptr.get()->body_ptr().get()->body2world_pose();

            auto translation = body2camera_pose.translation();
            Eigen::Quaternionf quaternionEigen (body2camera_pose.rotation());
            // Switch x,y,z and roll,pitch,yaw to be conform to the translation
            // and rotation vectors from the simulator

            geometry_msgs::PoseStamped estimatedObjectPoseRelativeToCameraLink;
            
            estimatedObjectPoseRelativeToCameraLink.header.stamp = msg_header.stamp;                   // when image was taken, look into image timestamp
            estimatedObjectPoseRelativeToCameraLink.header.frame_id = cameraSensorFrame; // msg_header.frame_id;

            estimatedObjectPoseRelativeToCameraLink.pose.position.x = translation.x();
            estimatedObjectPoseRelativeToCameraLink.pose.position.y = translation.y();
            estimatedObjectPoseRelativeToCameraLink.pose.position.z = translation.z();

            estimatedObjectPoseRelativeToCameraLink.pose.orientation.x = quaternionEigen.x();
            estimatedObjectPoseRelativeToCameraLink.pose.orientation.y = quaternionEigen.y();
            estimatedObjectPoseRelativeToCameraLink.pose.orientation.z = quaternionEigen.z();
            estimatedObjectPoseRelativeToCameraLink.pose.orientation.w = quaternionEigen.w();
            /*
            tf2::Vector3 positionRelativeToCameraLink;
            tf2::Quaternion quaternionRelativeToCameraLink;
	        tf2::fromMsg(estimatedObjectPoseRelativeToCameraLink.pose.orientation, quaternionRelativeToCameraLink);
            tf2::fromMsg(estimatedObjectPoseRelativeToCameraLink.pose.position, positionRelativeToCameraLink);

            tf2::Quaternion sensor_link_transform;
            tf::Matrix3x3 rotationRelativeToCameraFrame;
           // rotationRelativeToCameraFrame.getRotation(quaternionRelativeToCameraLink)
            sensor_link_transform.setRPY(-1.57, 0, 0);
            auto rotation =
                        sensor_link_transform * quaternionRelativeToCameraLink;

            auto trans =
                    sensor_link_transform * positionRelativeToCameraLink;
            double roll, pitch, yaw;
            //tf2::Quaternion quaternion;    
            //rotation.getRotation(quaternion);
            estimated_pose_msg::estimatedPose estimatedObjectPoseRelativeToCameraCV;
            estimatedObjectPoseRelativeToCameraCV.poseStamped.pose.position.x = trans.x();
            estimatedObjectPoseRelativeToCameraCV.poseStamped.pose.position.y = trans.y();
            estimatedObjectPoseRelativeToCameraCV.poseStamped.pose.position.z = trans.z();

            estimatedObjectPoseRelativeToCameraCV.poseStamped.pose.orientation.x = rotation.x();
            estimatedObjectPoseRelativeToCameraCV.poseStamped.pose.orientation.y = rotation.y();
            estimatedObjectPoseRelativeToCameraCV.poseStamped.pose.orientation.z = rotation.z();
            estimatedObjectPoseRelativeToCameraCV.poseStamped.pose.orientation.w = rotation.w();
            */

           // tf::Stamped<tf::Point> PointWRTcamera = {{translation.x(),translation.y(),translation.z()}, ros::Time(0), "CameraRight/camera_sensor_link"};
           // tf::Stamped<tf::Point> PointWRTBase;

            estimated_pose_msg::estimatedPose estimatedObjectPoseRelativeToBaseLink;
            estimatedObjectPoseRelativeToBaseLink.poseStamped = calculateTransformStampPosition (estimatedObjectPoseRelativeToCameraLink, tf_prefix + std::string("base_link"));
            estimatedObjectPoseRelativeToBaseLink.poseStamped.header.stamp = msg_header.stamp; // when image was taken, look into image timestamp
            estimatedObjectPoseRelativeToBaseLink.poseStamped.header.frame_id = msg_header.frame_id;
            estimatedObjectPoseRelativeToBaseLink.success = true;
            estimatedObjectPoseRelativeToBaseLink.trackedObjectName = referenced_body_ptr->name();
            estimated_pose_msg::estimatedPose estimatedObjectPoseRelativeToCameraSensor;
            estimatedObjectPoseRelativeToCameraSensor.poseStamped = estimatedObjectPoseRelativeToCameraLink;
            posePub->publish (estimatedObjectPoseRelativeToBaseLink);

            double roll, pitch, yaw;
            tf2::Matrix3x3 rotation;
            tf2::Quaternion quaternion;
	        tf2::fromMsg(estimatedObjectPoseRelativeToBaseLink.poseStamped.pose.orientation, quaternion);
            rotation.setRotation(quaternion);
            rotation.getRPY(roll, pitch, yaw);
            std::cout<<"\n roll "<< roll<<" pitch "<<pitch<<" yaw "<<yaw;
        }
        return true;
    }

    RosPublisher::RosPublisher (const std::string &name, ros::NodeHandle& nh) : Publisher{name} {setParameters(nh);}

    RosPublisher::RosPublisher (const std::string &name,
                                const std::filesystem::path &metafile_path, ros::NodeHandle& nh)
        : Publisher{name, metafile_path} {setParameters(nh);}

 }// namespace m3t
