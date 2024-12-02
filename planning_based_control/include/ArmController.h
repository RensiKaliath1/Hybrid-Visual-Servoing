
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/Marker.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ObstacleAvoidance.h>

class Controller
{
	public:
		struct ObjectPose
		{
			geometry_msgs::PoseStamped poseStamped;
			geometry_msgs::TransformStamped coordinateFrame;
			geometry_msgs::TransformStamped POIFrame; //Pose of the POI on the object with the Z-axis defining the angle of approach
			double accuracy; //In percentage (90-projError)/90 *100
			bool poseValidity = false;
		};
		ros::NodeHandle nh;
		Controller ();
		ObstacleAvoidance obstacleAvoidance;
		//virtual ~Controller ();
		std::string logFilename = "";
		int numberOFPlanningSuccesses = 0;
		geometry_msgs::PoseStamped interimTargetPoseWRTGripper;
		geometry_msgs::PoseStamped finalTargetPoseWRTGripper;
		geometry_msgs::PoseStamped searchTargetPose;

		enum State {WAITING_FOR_POSE, SEARCHING_FOR_OBJECT, INTERIM_PLANNING, INTERIM_EXECUTION,  EVALUATION, FINAL_PLANNING, FINAL_EXECUTION, FINISHED_EXECUTION};
		enum OperationType {INSPECT_STATIONARY, INSPECT_NON_STATIONARY, TOUCH, GRASP};
		enum TruncationType {HALF_TRAJECTORY, STEP_BY_STEP};
		//Member Functions
		State getState(); //Returns the current state of the controller
		bool planTrajectory(geometry_msgs::Pose pose);
		void setCurrentObjectPose(ObjectPose objectPose);
		std::vector<double>jointVariableVelocities;
		void SetJointVelocities(std::vector<double> jointVelocities);
		moveit::planning_interface::MoveGroupInterface moveGroupInterfaceArm{"arm"};
		moveit::planning_interface::MoveGroupInterface moveGroupInterfaceGripper{"gripper"};
		ros::ServiceClient planning_scene_diff_client;
		double timePrevious = 0.0;
		std::vector<double> actualJointPositions;
		void sendGoal(geometry_msgs::Pose pose);
		void RemoveTargetOBjectFromCollision(geometry_msgs::Pose pose);
		void RemoveOBjectFromCollision(geometry_msgs::Pose pose);
		void AddObjectsToCollision(geometry_msgs::Pose pose);
	private:

		//Parameters
		OperationType operationType = INSPECT_NON_STATIONARY;
		bool openLoopMode = false;
		bool orientationLock = false;
		double inspectionDistance = 0;
		float inspectionPoseErrorMargin = 0;
		float inspectionOrientationErrorMargin = 0;
		std::string gripperFrameId;
		std::string objectFrameId;
		TruncationType truncationType = STEP_BY_STEP;

		//Information Sting Generation Member Functions
		std::string getStateString(); //Returns the state of the controller in a string format
		std::string getOperationTypeString(); //Returns the operation type of the controller in a string format
	
		State state = WAITING_FOR_POSE;
		bool finalSequence = false;

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		//Action Client
		const std::string s = "gen3_joint_trajectory_controller/follow_joint_trajectory";
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectoryActionClient{s, true};

		//TF Buffer and Listener
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener {tfBuffer};
		tf2_ros::TransformBroadcaster tfBroadcaster;
		//Member Functions
		void planTrajectory();
		void setParameters(ros::NodeHandle& nh);
		void calculateInterimTargetPose(geometry_msgs::PoseStamped& currentObjectPose, geometry_msgs::TransformStamped currentObjectFrame);
		tf2::Quaternion findShortestRotation(geometry_msgs::TransformStamped& objectFrame);
		void truncateTrajectory();
		void sendGoalToTrajectoryActionClient();
		void goalCompletionCallback(const actionlib::SimpleClientGoalState & goalstate, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
		bool readyForFinalSequence();
		float estimateDistanceToObject();
		float estimateOrientationError();
		bool isInterimPoseReached(float distanceToObject, float angleErrorInPose);
		void performFinalSequence();
		void calculateFinalTargetPose(double perpendicularDistance);

		//Record and Update stats
		void writeToLog(std::string text);
};	

