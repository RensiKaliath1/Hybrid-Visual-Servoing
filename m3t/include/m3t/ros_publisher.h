#ifndef m3t_INCLUDE_m3t_ROS_PUBLISHER_H_
#define m3t_INCLUDE_m3t_ROS_PUBLISHER_H_

#include <m3t/body.h>
#include <m3t/link.h>
#include <m3t/camera.h>
#include <m3t/publisher.h>
#include <m3t/ros_camera.h>

// ROS
#include "ros/package.h"
#include <eigen_conversions/eigen_msg.h>
#include <estimated_pose_msg/estimatedPose.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
namespace m3t
{

    class RosPublisher : public Publisher
    {
    public:
        // Constructor
        RosPublisher (const std::string &name, ros::NodeHandle& nh);
        RosPublisher (const std::string &name,
                      const std::filesystem::path &metafile_path, ros::NodeHandle& nh);

        // Setters
        void set_name (const std::string &name);
        void set_metafile_path (const std::filesystem::path &metafile_path);

        // Class pointers to get to all necessary data
        bool AddReferencedBody (const std::shared_ptr<Link> &referenced_body_ptr);
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
        void setParameters(ros::NodeHandle& nh);
    protected:
        // Variables
        std::string name_{};
        std::filesystem::path metafile_path_{};
        bool set_up_ = false;

    private:
        std::vector<std::shared_ptr<Link>> referenced_body_ptrs_{};
        std::shared_ptr<Camera> camera_ptr_ = nullptr;
        ros::Publisher *posePub = nullptr;
        tf::TransformListener tfListener;
        std::string cameraSensorFrame;

        std::string tf_prefix;
        geometry_msgs::PoseStamped calculateTransformStampPosition (const geometry_msgs::PoseStamped &geoPoseStampedIn, const std::string targetFrame);
    };

} // namespace m3t

#endif // m3t_INCLUDE_m3t_ROS_PUBLISHER_H_
