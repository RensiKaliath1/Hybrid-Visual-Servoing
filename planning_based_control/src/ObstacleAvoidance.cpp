#include <ObstacleAvoidance.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

namespace {

    // Helper function to create and initialize a CollisionObject with a given id, shape, and pose
    moveit_msgs::CollisionObject createCollisionObject(const std::string& id, const geometry_msgs::Pose& pose, const std::vector<double>& dimensions) {
        moveit_msgs::CollisionObject collision_object;
        collision_object.id = id;
        collision_object.header.frame_id = "world"; // Assuming "world" frame, update if necessary
        
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = dimensions;
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }

    // Helper function to create an AttachedCollisionObject for attaching to the gripper
    moveit_msgs::AttachedCollisionObject createAttachedCollisionObject(const std::string& link_name, const geometry_msgs::Pose& pose) {
        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = link_name;
        attached_object.object.header.frame_id = link_name;
        attached_object.object.id = "box";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.1, 0.1, 0.5}; // Example dimensions for box

        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(pose);

        return attached_object;
    }
}

void ObstacleAvoidance::RemoveTargetObjectFromCollision(geometry_msgs::Pose pose) {
    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
    planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_model_loader);

    // Create collision objects
    moveit_msgs::CollisionObject collision_object1 = createCollisionObject("asymmetric_pipestar", pose, {0.1, 2, 2});
    geometry_msgs::Pose box_pose2;
    box_pose2.orientation.w = pose.orientation.w;
    box_pose2.position.x = 0;
    box_pose2.position.y = -0.2;
    box_pose2.position.z = 0;
    moveit_msgs::CollisionObject collision_object2 = createCollisionObject("asymmetric_pipestar", box_pose2, {2, 0.1, 2});

    std::vector<moveit_msgs::CollisionObject> collision_objects = {collision_object1, collision_object2};
    planning_scene_interface.applyCollisionObjects(collision_objects);

    // Allow collision between gripper and the box for grasping
    planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix& acm = ls->getAllowedCollisionMatrixNonConst();
    acm.setEntry("asymmetric_pipestar", "robotiq_85_left_finger_tip_link", true);
    acm.setEntry("asymmetric_pipestar", "robotiq_85_right_finger_tip_link", true);

    // Apply the new planning scene
    moveit_msgs::PlanningScene diff_scene;
    ls->getPlanningSceneDiffMsg(diff_scene);
    planning_scene_interface.applyPlanningScene(diff_scene);

    // Sleep to allow the scene to update
    ros::Duration(0.1).sleep();

    // Log available planning groups
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(moveGroupInterfaceGripper.getJointModelGroupNames().begin(),
              moveGroupInterfaceGripper.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

void ObstacleAvoidance::RemoveObjectFromCollision(geometry_msgs::Pose pose) {
    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
    planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_model_loader);

    // Create collision objects for asymmetric_pipestar and pen
    moveit_msgs::CollisionObject collision_object1 = createCollisionObject("asymmetric_pipestar", pose, {0.2, 0.2, 0.2});
    auto endeffectorPose = moveGroupInterfaceArm.getCurrentPose(moveGroupInterfaceArm.getEndEffector());
    geometry_msgs::Pose box_pose1 = endeffectorPose.pose;
    moveit_msgs::CollisionObject collision_object2 = createCollisionObject("pen", box_pose1, {0.2, 0.2, 0.2});

    // Apply the collision objects to the scene
    std::vector<moveit_msgs::CollisionObject> collision_objects = {collision_object1, collision_object2};
    planning_scene_interface.applyCollisionObjects(collision_objects);

    // Sleep to allow the scene to update
    ros::Duration(0.1).sleep();
}

void ObstacleAvoidance::AddObjectsToCollision(geometry_msgs::Pose pose) {
    // Wait for the planning scene to exist
    planning_scene_diff_client.waitForExistence();

    // Create and attach the collision object
    moveit_msgs::AttachedCollisionObject attached_object = createAttachedCollisionObject("robotiq_85_left_finger_tip_link", pose);
    
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;

    // Apply the planning scene with the attached object
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}