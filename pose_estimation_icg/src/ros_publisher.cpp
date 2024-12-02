#include <icg/ros_publisher.h>

namespace icg
{

    geometry_msgs::PoseStamped RosPublisher::calculateTransformStampPosition (const geometry_msgs::PoseStamped &geoPoseStampedIn, const std::string targetFrame = "GETjag/base_link")
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
        const std::shared_ptr<Body> &referenced_body_ptr)
    {
        set_up_ = false;
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
                                    referenced_body_ptr->body2world_pose();

            auto translation = body2camera_pose.translation();
            Eigen::Quaternionf quaternionEigen (body2camera_pose.rotation());
            // Switch x,y,z and roll,pitch,yaw to be conform to the translation
            // and rotation vectors from the simulator

            geometry_msgs::PoseStamped estimatedObjectPoseRelativeToCameraLink;
            estimatedObjectPoseRelativeToCameraLink.header.stamp = msg_header.stamp;                   // when image was taken, look into image timestamp
            estimatedObjectPoseRelativeToCameraLink.header.frame_id = "camera_rgb_optical_frame"; // msg_header.frame_id;

            estimatedObjectPoseRelativeToCameraLink.pose.position.x = translation.x();
            estimatedObjectPoseRelativeToCameraLink.pose.position.y = translation.y();
            estimatedObjectPoseRelativeToCameraLink.pose.position.z = translation.z();

            estimatedObjectPoseRelativeToCameraLink.pose.orientation.x = quaternionEigen.x();
            estimatedObjectPoseRelativeToCameraLink.pose.orientation.y = quaternionEigen.y();
            estimatedObjectPoseRelativeToCameraLink.pose.orientation.z = quaternionEigen.z();
            estimatedObjectPoseRelativeToCameraLink.pose.orientation.w = quaternionEigen.w();

            estimated_pose_msg::estimatedPose estimatedObjectPoseRelativeToCameraCV;
            estimatedObjectPoseRelativeToCameraCV.poseStamped =  estimatedObjectPoseRelativeToCameraLink;//calculateTransformStampPosition (estimatedObjectPoseRelativeToCameraLink, "camera_sensor_link");
            estimatedObjectPoseRelativeToCameraCV.poseStamped.header.stamp = msg_header.stamp; // when image was taken, look into image timestamp
            estimatedObjectPoseRelativeToCameraCV.poseStamped.header.frame_id = msg_header.frame_id;
            estimatedObjectPoseRelativeToCameraCV.success = true;
            estimatedObjectPoseRelativeToCameraCV.trackedObjectName = referenced_body_ptr->name();
            posePub->publish (estimatedObjectPoseRelativeToCameraCV);

            tf::Matrix3x3 rotation;
            tf::Quaternion quaternion;

            quaternion.setX(0.9122587442398071);
            quaternion.setY(0.009011730551719666);       
            quaternion.setZ(0.013047372922301292);
            quaternion.setW(0.4093073308467865);   

            quaternion.setX(0.7548447847366333);
            quaternion.setY(-0.654657244682312);       
            quaternion.setZ(-0.039410777390003204);
            quaternion.setW(-0.0089441686868667);   

            quaternion.setX(0.9095437526702881);
            quaternion.setY(-0.02218005433678627);       
            quaternion.setZ(0.012410858646035194);
            quaternion.setW(0.41483038663864136);   
            quaternion.setX(-0.667621374130249);
            quaternion.setY(0.6731553673744202);       
            quaternion.setZ(-0.20952485501766205);
            quaternion.setW(-0.2392546832561493);
                   
            rotation.setRotation(quaternion);
            double roll, pitch, yaw;
            rotation.getRPY(roll, pitch, yaw);
            std::cout<<"\n---------- desired roll pitch yaw is------"<< roll<<" "<<pitch<<" "<<yaw;

        }
        return true;
    }

    RosPublisher::RosPublisher (const std::string &name) : Publisher{name} {}

    RosPublisher::RosPublisher (const std::string &name,
                                const std::filesystem::path &metafile_path)
        : Publisher{name, metafile_path} {}

} // namespace icg
