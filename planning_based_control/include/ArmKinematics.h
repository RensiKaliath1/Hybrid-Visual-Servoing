#include <cmath>
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <vector>
#include <ArmController.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <estimated_pose_msg/estimatedPose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>


class ArmKinematics
{
public:
        ArmKinematics(ros::NodeHandle& nh);
        Controller controller;
        void GetJointValues(); 
        void GetForwardkinematics();
        tf2::Quaternion FindShortestRotation();
        void ConvertVelocityFromObjectToRobotBaseFrame(geometry_msgs::Twist msg, tf2::Vector3 objectPositionWRTCamera);
        geometry_msgs::PoseStamped ChangeBaseFrame( std::string target_frame, geometry_msgs::PoseStamped pose_to_be_transformed);
        void ObjectDetectedCallBack(const std_msgs::Bool& isObjectDetected);      
        geometry_msgs::PointStamped ChangeBaseFramePoint( std::string target_frame, geometry_msgs::PointStamped point_to_be_transformed, tf2_ros::Buffer &tf_buffer);
        geometry_msgs::PoseStamped calculateTransformStampPosition (const geometry_msgs::PoseStamped &geoPoseStampedIn, const std::string targetFrame);
        void ObjectPoseBroadcaster(const estimated_pose_msg::estimatedPose& msg);
        void SetObjectPose(const estimated_pose_msg::estimatedPose& msg);
        Eigen::MatrixXd GetJacobian(Eigen::Vector3d reference_point_position);
        ros::Publisher JointVelocityPublisher;
        ros::Publisher JointStatePublisher;
        ros::Subscriber targetObjectSubscriber;
        ros::Subscriber targetObjectSubscriberBroadcast;
        ros::Subscriber armSubscriber;
        ros::Subscriber IsObjectDetectedSubscriber;
        tf::TransformListener listener;
        tf::TransformBroadcaster broadcasterTargetObject;
        tf::TransformBroadcaster broadcasterDesiredEndEffectorPose;

  	tf2_ros::Buffer tfBuffer;
  	tf2_ros::TransformListener tfListener {tfBuffer};

        std::string operation;
        std::string servo_type;
        ros::Publisher integerPublisher;
        std::vector<double> desiredTransform;

private:
        std::string tf_prefix ="";
        void SetParameters(ros::NodeHandle& nh);
        const robot_state::JointModelGroup* joint_model_group;
        robot_model::RobotModelPtr kinematic_model;
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface>moveGroup;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>planningSceneInterface;
};
