## A Hybrid Visual Servoing Approach to Improve Robot Dexterity

A Position based visual servoing or a planning-based control approach is used to approach the target object. Third party library M3T and ICG is used for 
pose estimation. MoveIt is used for controlling the arm by planning a trajectory towards the target object. For planning-based control a eye-to-hand 
camera configuration is used. Moveit calibration package calibrates the pose of the eye-to-hand camera w.r.t base of the arm. 

As the arm moves towards the object, the eye-in-hand camera loses tracking and image-based visual servoing takes control. For feature detection, SURF (Speeded Up Robust Features) and homography based tracking is employed. 

### Result of inspection task performed by Kinova Gen3 arm inside a pipestar object with texture on it
![Screenshot 2024-11-25 132302](https://github.com/user-attachments/assets/37efc1be-6033-4b91-81a6-0ee708e2f0c0)

<br />
Third party libraries used: <br />
1. M3T <br />
2. ICG <br />
3. OpenCV <br />
4. MoveIt <br />
5. Visp <br />

Open-loop planning based approach is implemented in package planning_based_controller
PBVS, IBVS and SURF-Homography tracking in package visual_servoing
Files m3t_for_non_kinematic_structures.cpp has m3t with ros wrapper to run with other nodes. This file is used for non-kinematic structure tracking.
File MainRos.cpp is used for kinematic structure tracking, Two threads run where one thread is for tracking kinematic structure and another for non-kinematic structure.

### start kinova_arm driver:
	roslaunch kortex_driver kortex_driver_get.launch
### start eye-in-hand camera driver
	roslaunch kinova_vision kinova_vision.launch
### To run IBVS:
	Go to visual-servo_params.yaml and choose servo_type as IBVS, initial approach is not considered in this case. Choose the task to be performed 
	
### To run PBVS:
	Go to visual-servo_params.yaml and choose servo_type as PBVS, initial approach is not considered in this case. Choose the task to be performed (touch, inspect, grasp, follow).
	
### To run Plannning-based:
	Go to visual-servo_params.yaml and choose servo_type as PlanningBased, initial approach is not considered in this case. Choose the task to be performed (touch, inspect, grasp, follow).
	roslaunch visual_servoing visual_servo.launch
	
### To run Hybrid pbvs-ibvs system
	start pbvs-ibvs system:
		Go to visual-servo_params.yaml and choose servo_type as hybrid, initial approach as PBVS and choose the task to be performed.
		roslaunch visual_servoing visual_servo.launch
		
### To run hybrid planning-based-ibvs system:
	start eye-to-hand camera:
		roslaunch openni2_launch openni2.launch
	start planning_based-ibvs system:
		Go to visual-servo_params.yaml and choose servo_type as hybrid, initial approach as PBVS and choose the task to be performed.
		roslaunch visual_servoing visual_servo.launch	

For IBVS a template image has to be provided as input, taking the image of the object with the kinova eye-in-hand camera provides better tracking results.
For PBVS, the object origin is considered to be at the center of pipestar
For IBVS, The object origin is the center of the plane whose corners are being tracked

### To run ibvs in Gazebo simulator:
	rosrun ibvs_veloctiy ibvs_automatic_init
	
### To track kinematic structures, run node posetracking_kinematic
	first eye-to-hand camera has to be calibrated and the parameters in config file m3t/config/dexterity_params.yaml must be adjusted accordingly.
	roslaunch m3t convertTomatrix.launch
	rosrun m3t posetracking_kinematic
	
### To calibrate eye-to-hand camera:
	In RVIZ, add moveit_calibration_gui HandEyeCalibration and calibrate the camera. Attach Aruco target to the end-effector. 
	Make sure the eye_to_hand camera pose w.r.t robot base is being broadcasted. Now eye-to-hand camera is ready to be used.




