#include<ArmKinematics.h>
#include <algorithm>
#include <ros/init.h>
#include <std_msgs/Float64.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>

ArmKinematics::ArmKinematics(ros::NodeHandle& nh)
{    
	SetParameters(nh);
	listener.waitForTransform(tf_prefix + "tool_frame", "object_frame", ros::Time(0), ros::Duration(1));
    targetObjectSubscriberBroadcast = nh.subscribe("/tracked_object_poses", 1, &ArmKinematics::ObjectPoseBroadcaster, this);
	IsObjectDetectedSubscriber = nh.subscribe("/is_object_detected_ibvs", 10, &ArmKinematics::ObjectDetectedCallBack, this);
	integerPublisher = nh.advertise<std_msgs::Float64>("/my_gen3/in/distanceToObject",1);
}

void ArmKinematics::ObjectDetectedCallBack(const std_msgs::Bool& isObjectDetected)
{
	if(isObjectDetected.data == true)
		ros::shutdown();
}

void ArmKinematics::SetParameters(ros::NodeHandle& nh)
{
    std::string tf_prefix_param;
	std::string initial_approach;
    nh.getParam("/Dexterity/hardware_parameters/tf_prefix", tf_prefix_param);
    tf_prefix = tf_prefix_param;
	nh.getParam("/VisualServoing/parameters/servo_type", servo_type);
    nh.getParam("/VisualServoing/parameters/task", operation);
	nh.getParam("/VisualServoing/parameters/initial_approach", initial_approach);
	if(initial_approach != "PlanningBased")
		ros::shutdown();
    if(operation == "touch")
    
	nh.getParam("/VisualServoing/parameters/PlanningBased/touch/desired_camera_transform_wrt_object_frame", desiredTransform);
    if(operation == "inspection")
	    nh.getParam("/VisualServoing/parameters/PlanningBased/inspection/desired_camera_transform_wrt_object_frame", desiredTransform);
    if(operation == "graspSide")
	    nh.getParam("/VisualServoing/parameters/PlanningBased/graspSide/desired_camera_transform_wrt_object_frame", desiredTransform);
    if(operation == "graspTop")
	    nh.getParam("/VisualServoing/parameters/PlanningBased/graspTop/desired_camera_transform_wrt_object_frame", desiredTransform);
}

geometry_msgs::PoseStamped ArmKinematics::calculateTransformStampPosition (const geometry_msgs::PoseStamped &geoPoseStampedIn, const std::string targetFrame)
{
    geometry_msgs::PoseStamped geoPoseStampedOut;
    try
    {
       // listener.waitForTransform("/camera_base_link", targetFrame, ros::Time(0), ros::Duration(1));
        listener.transformPose (targetFrame, geoPoseStampedIn, geoPoseStampedOut);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN ("%s", ex.what());
        ros::Duration (1.0).sleep();
    }
    return geoPoseStampedOut;
}

void ArmKinematics::ObjectPoseBroadcaster(const estimated_pose_msg::estimatedPose& msg)
{
	auto poseWrtToolFrameMsg = calculateTransformStampPosition(msg.poseStamped, "kinova_gen3_base_link");
    tf2::Vector3 positionRelativeToBaseLink;
    tf2::Quaternion quaternionRelativeToBaseLink;
	tf2::fromMsg(poseWrtToolFrameMsg.pose.orientation, quaternionRelativeToBaseLink);
    tf2::fromMsg(poseWrtToolFrameMsg.pose.position, positionRelativeToBaseLink);

    tf::Quaternion quaternion(quaternionRelativeToBaseLink.x(), quaternionRelativeToBaseLink.y(), quaternionRelativeToBaseLink.z(), quaternionRelativeToBaseLink.w());
    broadcasterTargetObject.sendTransform(tf::StampedTransform(tf::Transform(quaternion, 
    tf::Vector3(positionRelativeToBaseLink[0], positionRelativeToBaseLink[1], positionRelativeToBaseLink[2])),
    ros::Time::now(),tf_prefix + "kinova_gen3_base_link", "object_frame"));  

	tf::Quaternion quaternionDesiredFrame;
	tf::Vector3 translationDesiredFrame(desiredTransform[0], desiredTransform[1], desiredTransform[2]);
	quaternionDesiredFrame.setRPY(desiredTransform[3], desiredTransform[4], desiredTransform[5]);
	broadcasterDesiredEndEffectorPose.sendTransform(tf::StampedTransform(tf::Transform(quaternionDesiredFrame, 
    translationDesiredFrame),
    ros::Time::now(), "object_frame", "desiredEndEffectorPose"));  

    static bool first = true;
    if(first)
    {
        ros::Duration(1.0).sleep();
        first = false;
    }
    SetObjectPose(msg); 
}

void ArmKinematics::SetObjectPose(const estimated_pose_msg::estimatedPose& msg)
{

    geometry_msgs::Transform targetPoseWRTBase;
	
	try
	{
    	auto targetGoemetryMsgWRTBase = tfBuffer.lookupTransform("kinova_gen3_base_link", "desiredEndEffectorPose", ros::Time(0));
		targetPoseWRTBase = targetGoemetryMsgWRTBase.transform;

    	if((controller.getState() == Controller::WAITING_FOR_POSE))
    	{
        	geometry_msgs::PoseStamped desiredPose;
        	desiredPose.pose.position.x = targetPoseWRTBase.translation.x;
        	desiredPose.pose.position.y = targetPoseWRTBase.translation.y;
        	desiredPose.pose.position.z = targetPoseWRTBase.translation.z;

        	desiredPose.pose.orientation.x = targetPoseWRTBase.rotation.x;
        	desiredPose.pose.orientation.y = targetPoseWRTBase.rotation.y;
        	desiredPose.pose.orientation.z = targetPoseWRTBase.rotation.z;
        	desiredPose.pose.orientation.w = targetPoseWRTBase.rotation.w;
			
        	bool success = controller.planTrajectory(desiredPose.pose);
			if(success)
			{
				std_msgs::Float64 distance;
				distance.data = desiredPose.pose.position.z;
				integerPublisher.publish(distance);
				ros::shutdown();
			}

		}
	}
	catch (tf::TransformException &ex)
    {
        ROS_WARN ("%s", ex.what());
        ros::Duration (1.0).sleep();
    }
}

