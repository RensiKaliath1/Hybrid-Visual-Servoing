#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
class ObstacleAvoidance
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface moveGroupInterfaceArm{"arm"};
    moveit::planning_interface::MoveGroupInterface moveGroupInterfaceGripper{"gripper"};
    ros::ServiceClient planning_scene_diff_client;
    public: 
            void RemoveTargetOBjectFromCollision(geometry_msgs::Pose pose);
            void RemoveOBjectFromCollision(geometry_msgs::Pose pose);
            void AddObjectsToCollision(geometry_msgs::Pose pose);
};