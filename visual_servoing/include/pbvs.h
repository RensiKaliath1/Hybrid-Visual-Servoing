#include <vector>

#include <ros/ros.h>
#include <ros/init.h>
#include <ros/publisher.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include <visp_bridge/image.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include "visp3/core/vpHomogeneousMatrix.h"
#include "visp3/core/vpRxyzVector.h"
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <kinova_twist/TwistCommand.h>
#include <kortex_driver/Base_JointSpeeds.h>
#include <kortex_driver/JointSpeed.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <estimated_pose_msg/estimatedPose.h>

class PBVS{
public:
	ros::NodeHandle n;
	geometry_msgs::Twist start_twist;
	geometry_msgs::Pose start_pose;

	gazebo_msgs::ModelState modelstate;
	gazebo_msgs::GetModelState getmodelstate_image_4pts;
	gazebo_msgs::GetModelState getmodelstate_image_4pts_world;
	gazebo_msgs::GetModelState getmodelstate_cam;
	gazebo_msgs::SetModelState setmodelstate_cam;

	ros::ServiceClient client ;
	ros::ServiceClient client_cam ;
	ros::ServiceClient client_image_4pts ;
	ros::ServiceClient client_image_4pts_world;

	geometry_msgs::Quaternion image_4pts_orientation;
	geometry_msgs::Point image_4pts_position ;
	geometry_msgs::Quaternion  cam_orientation ;
	geometry_msgs::Point cam_position;
	geometry_msgs::Point image_4pts_position_world;

    vpServo task;
    tf::TransformListener listener;

    tf::TransformBroadcaster broadcasterTargetObject;
    tf::TransformBroadcaster broadcasterDesiredEndEffectorPose; 
 
    vpHomogeneousMatrix cdMc; // Transformation between desired and current camera frame
    vpRotationMatrix cdRc;    // Rotation between desired and current camera frame
    vpRotationMatrix cRcd;    // Rotation between current and desired camera frame
    double lambda = 1;
    double adaptive_gain_zero = 1;
    double adaptive_gain_inf = 0.5;
    double error_threshold = 0;
    double tracking_error_threshold = 0;
    double previous_error = 0;
    int counter = 0;
    // Compute the transformation from the initial camera position to the
    // desired one
    vpHomogeneousMatrix cMcd;
    vpHomogeneousMatrix cMo;
    vpPoseVector cd_r_o;
            
    vpFeatureTranslation t; 
    // Build the 3D rotation feature: thetaU_cRc*
    vpFeatureThetaU tu; // current feature

    // Sets the desired rotation (always zero !)  since s is the
    // rotation that the camera has to achieve. Here s* = (0, 0)^T
    vpFeatureTranslation td; 
    vpFeatureThetaU tud;// desired feature

    std::string servo_type;
    std::string operation;
    bool simulator;
    double touch_distance;
    double inspection_distance;
    std::vector<double> desiredTransform;
    std::string modelParametersFilePath;
    bool stopRequestFromIBVS = false;
    vpServo::vpServoIteractionMatrixType interactionMatrixType = vpServo::DESIRED;
    PBVS();
    void ShutDownNodeAndSetKinovaVelocityToZero();
    void positionCallback(const estimated_pose_msg::estimatedPose& msg);
    void ObjectDetectedCallBack(const std_msgs::Bool& isObjectDetected);
    void SendVelocityToKinematicsNode(vpColVector velocityObjectFrame, tf::Vector3 pointTranslation);
    void SetParameters(ros::NodeHandle& n);
    void SendAndReceiveSimulator();
    void SendVelocityToGazebo(vpColVector v);
 	ros::Subscriber imageSub;
 	ros::Subscriber IsObjectDetectedSubscriber;
    ros::Publisher velocityPublisher;
    ros::Publisher distancePublisher;
    ros::Publisher m3tShutdownPublisher;
    int index = 0;
};