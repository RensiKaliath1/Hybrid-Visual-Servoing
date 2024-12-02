#include<ArmController.h>
#include <ros/init.h>

Controller::State Controller::getState()
{
	return state;
}

std::string Controller::getStateString()
{
	  const char* controllerStates[] = {"WAITING_FOR_POSE", "SEARCHING_FOR_OBJECT", "INTERIM_PLANNING", "INTERIM_EXECUTION", "EVALUATION", "FINAL_PLANNING", "FINAL_EXECUTION", "FINISHED_EXECUTION"};
	  return controllerStates[state];
}

std::string Controller::getOperationTypeString()
{
	const char* operationTypes[] = {"INSPECT_STATIONARY", "INSPECT_NON_STATIONARY", "TOUCH", "GRASP"};
	return operationTypes[operationType];
}


bool Controller::planTrajectory(geometry_msgs::Pose pose)
{
    state = FINAL_PLANNING;
    int approaching_scale = 0;
	moveGroupInterfaceArm.setPoseTarget(pose);	
    moveit::planning_interface::MoveGroupInterface::Plan plan;
   	if (moveGroupInterfaceArm.plan (plan) == moveit::core::MoveItErrorCode::SUCCESS)
	{
		const int vectorSize = plan.trajectory_.joint_trajectory.points.size();
		moveGroupInterfaceArm.execute(plan);
		state = WAITING_FOR_POSE;
		return true;
	}
	else 
	{
		return false;
	}
}
