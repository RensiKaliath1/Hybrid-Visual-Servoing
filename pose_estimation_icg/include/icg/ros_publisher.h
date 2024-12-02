#ifndef ICG_INCLUDE_ICG_ROS_PUBLISHER_H_
#define ICG_INCLUDE_ICG_ROS_PUBLISHER_H_

#include <icg/body.h>
#include <icg/camera.h>
#include <icg/publisher.h>
#include <icg/ros_camera.h>

// ROS
#include "ros/package.h"
#include <eigen_conversions/eigen_msg.h>
#include <estimated_pose_msg/estimatedPose.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace icg
{

    class RosPublisher : public Publisher
    {
    public:
        // Constructor
        RosPublisher (const std::string &name);
        RosPublisher (const std::string &name,
                      const std::filesystem::path &metafile_path);

        // Setters
        void set_name (const std::string &name);
        void set_metafile_path (const std::filesystem::path &metafile_path);

        // Class pointers to get to all necessary data
        bool AddReferencedBody (const std::shared_ptr<Body> &referenced_body_ptr);
        bool DeleteReferencedBody (const std::string &name);
        void ClearReferencedBodies();
        void AddRosPublisher (ros::Publisher &pose_pub);
        void AddCamera (const std::shared_ptr<Camera> &camera_ptr);

        // Main methods
        bool SetUp() override;
        bool UpdatePublisher (int iteration) override;

        // Getters
        const std::string &name() const;
        const std::filesystem::path &metafile_path() const;
        bool set_up() const;

        std_msgs::Header msg_header;

    protected:
        // Variables
        std::string name_{};
        std::filesystem::path metafile_path_{};
        bool set_up_ = false;

    private:
        std::vector<std::shared_ptr<Body>> referenced_body_ptrs_{};
        std::shared_ptr<Camera> camera_ptr_ = nullptr;
        ros::Publisher *posePub = nullptr;
        tf::TransformListener tfListener;
        geometry_msgs::PoseStamped calculateTransformStampPosition (const geometry_msgs::PoseStamped &geoPoseStampedIn, const std::string targetFrame);
    };

} // namespace icg

#endif // ICG_INCLUDE_ICG_ROS_PUBLISHER_H_
