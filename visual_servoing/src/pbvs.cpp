#include <ros/init.h>

#include "visp3/core/vpColVector.h"
#include "visp3/vs/vpServo.h"

#include "pbvs.h"

PBVS::PBVS() :t(vpFeatureTranslation::cMcd), tu(vpFeatureThetaU::cRcd), td(vpFeatureTranslation::cMcd), tud(vpFeatureThetaU::cRcd)
{
    listener.waitForTransform("camera_link", "camera_sensor_link", ros::Time(0), ros::Duration(1));
    m3tShutdownPublisher =  n.advertise<std_msgs::Bool>("/shutdownM3T", 1);
    SetParameters(n);

    if(simulator == true)
        SendAndReceiveSimulator();
    else
        velocityPublisher = n.advertise<kinova_twist::TwistCommand>("/my_gen3/in/cartesian_velocity",1);

	image_transport::ImageTransport imageTransport (n);
	imageSub = n.subscribe ("/tracked_object_poses", 1, &PBVS::positionCallback, this);
    IsObjectDetectedSubscriber = n.subscribe("/is_object_detected_ibvs", 10, &PBVS::ObjectDetectedCallBack, this);
    integerPublisher = n.advertise<std_msgs::Float64>("/my_gen3/in/distanceToObject",1);
}

void PBVS::SetParameters(ros::NodeHandle& n)
{
    std::string initial_approach;
    n.getParam("/VisualServoing/parameters/servo_type", servo_type);
    n.getParam("/VisualServoing/parameters/task", operation);
    n.getParam("/VisualServoing/parameters/initial_approach", initial_approach);
    n.getParam("/VisualServoing/parameters/simulator", simulator);
    n.getParam("/VisualServoing/parameters/touch_distance", touch_distance);
    n.getParam("/VisualServoing/parameters/inspection_distance", inspection_distance);
	n.getParam("/VisualServoing/parameters/adaptive_gain_zero_pbvs", adaptive_gain_zero);
	n.getParam("/VisualServoing/parameters/adaptive_gain_inf_pbvs", adaptive_gain_inf);
	n.getParam("/VisualServoing/parameters/error_threshold", error_threshold);
    n.getParam("/VisualServoing/parameters/tracking_error_threshold", tracking_error_threshold);
    if(operation == "touch")
	    n.getParam("/VisualServoing/parameters/PBVS/touch/desired_camera_transform_wrt_object_frame", desiredTransform);
    if(operation == "inspection")
	    n.getParam("/VisualServoing/parameters/PBVS/inspection/desired_camera_transform_wrt_object_frame", desiredTransform);
    if(operation == "graspSide")
	    n.getParam("/VisualServoing/parameters/PBVS/graspSide/desired_camera_transform_wrt_object_frame", desiredTransform);
    if(operation == "graspTop")
	    n.getParam("/VisualServoing/parameters/PBVS/graspTop/desired_camera_transform_wrt_object_frame", desiredTransform);
   
    if(servo_type == "IBVS")
    {
        std_msgs::Bool dat;
        dat.data = true;
        m3tShutdownPublisher.publish(dat);
        ros::shutdown();
    }
    if(initial_approach == "PlanningBased")
        ros::shutdown();
}

//needed only if running in simulator
void PBVS::SendAndReceiveSimulator()
{
    client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	client_cam = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	client_image_4pts = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	client_image_4pts_world = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	getmodelstate_image_4pts.request.model_name = "pipestar";
	getmodelstate_image_4pts.request.relative_entity_name = "cameraStereo";

	getmodelstate_image_4pts_world.request.model_name = "pipestar";
	getmodelstate_image_4pts_world.request.relative_entity_name = "pipestar";

	getmodelstate_cam.request.model_name = "cameraStereo";
	getmodelstate_cam.request.relative_entity_name = "ground_plane";

	modelstate.model_name = (std::string) "cameraStereo";
	modelstate.reference_frame = (std::string) "cameraStereo";

}

void PBVS::ObjectDetectedCallBack(const std_msgs::Bool& isObjectDetected)
{
	if(isObjectDetected.data == true)
		stopRequestFromIBVS = true;
}

void PBVS::ShutDownNodeAndSetKinovaVelocityToZero()
{
		kinova_twist::TwistCommand kinovaVelocity;
		kinovaVelocity.reference_frame = 0;
		kinovaVelocity.twist.linear_x = 0 ;
		kinovaVelocity.twist.linear_y = 0 ;
		kinovaVelocity.twist.linear_z = 0 ;
		kinovaVelocity.twist.angular_x = 0;
		kinovaVelocity.twist.angular_y = 0; 
		kinovaVelocity.twist.angular_z = 0;
	   	velocityPublisher.publish(kinovaVelocity);
		cv::destroyAllWindows();
	 	ros::shutdown();
}

void PBVS::positionCallback(const estimated_pose_msg::estimatedPose& msg)
{
    try{
        if(stopRequestFromIBVS)
            ros::shutdown();

	    if(client_cam.call(getmodelstate_cam))
        {
            geometry_msgs::Quaternion orientation = getmodelstate_cam.response.pose.orientation;
	        tf::Quaternion quart = tf::Quaternion(orientation.x, orientation.y,orientation.z, orientation.w);
        }

        tf2::Vector3 positionRelativeToCamera;
        tf2::Quaternion quaternionRelativeToCamera;
        tf2::fromMsg(msg.poseStamped.pose.position, positionRelativeToCamera);
        tf2::fromMsg(msg.poseStamped.pose.orientation, quaternionRelativeToCamera);

        tf::Quaternion quaternion;
        quaternion.setX(quaternionRelativeToCamera.getX());
        quaternion.setY(quaternionRelativeToCamera.getY());       
        quaternion.setZ(quaternionRelativeToCamera.getZ());
        quaternion.setW(quaternionRelativeToCamera.getW());
        tf::Matrix3x3 rot;
        rotation.setRotation(quaternion);

        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);
        vpHomogeneousMatrix cdMo = vpHomogeneousMatrix(desiredTransform[0], desiredTransform[1], desiredTransform[2],desiredTransform[3], desiredTransform[4], desiredTransform[5]);
 
        rotation.getRPY(roll, pitch, yaw);
        cMo = vpHomogeneousMatrix(positionRelativeToCamera.getX(),  positionRelativeToCamera.getY(), positionRelativeToCamera.getZ(),
                                 roll, pitch, yaw);
        if(index == 0)
        {
            cMcd = cMo * cdMo.inverse();
            t.buildFrom(cMcd);
            tu.buildFrom(cMcd); 
            // Add the current and desired visual features
            task.addFeature(t, td);   // 3D translation
            task.addFeature(tu, tud); // 3D rotation theta u
            index++;
        }
        //set adaptive gain
        vpAdaptiveGain lambda(adaptive_gain_zero, adaptive_gain_inf, 30);   // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
        task.setLambda(lambda);

        // velocity of eye-in-hand camera is calculated
        task.setServo(vpServo::EYEINHAND_CAMERA);
        task.setInteractionMatrixType(interactionMatrixType);
        cMcd = cMo * cdMo.inverse();
        // Update the current visual features
        t.buildFrom(cMcd);
        tu.buildFrom(cMcd);
                
        vpColVector velocity;
        // Compute the control law
        velocity = task.computeControlLaw();
        if(simulator != true)
            SendVelocityToKinematicsNode(velocity, tf::Vector3{0,0,0});
        else
            SendVelocityToGazebo(velocity);

        // Retrieve the error
        double error = task.getError().sumSquare() ;
        std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;
        if (error < error_threshold)
	    {
		    std::cout<<"\n\nDesired pose is reached";
		    std_msgs::Float64 z_value;
            z_value.data = positionRelativeToCamera.getZ();
            integerPublisher.publish(z_value);
            ShutDownNodeAndSetKinovaVelocityToZero();
	    }
        previous_error = error;
        index++;
        }
        catch (const vpException &e) {
        std::cout << "Catch a ViSP exception: " << e << std::endl;
        return;
    }
}

void PBVS::SendVelocityToKinematicsNode(vpColVector velocityObjectFrame, tf::Vector3 pointTranslation)
{
	geometry_msgs::Twist velocity;
	tf::Vector3 out_rot;
	tf::Vector3 out_vel;
	geometry_msgs::Twist interframe_twist;
    tf::StampedTransform transform;
    
    // CAMERA_COLOR_FRAME AND TOOL FRAME link do not have have the same orientation
    tf::Stamped<tf::Vector3> velVectorCameraFrame {{velocityObjectFrame[0], velocityObjectFrame[1], velocityObjectFrame[2]}, ros::Time(0), "kinova_gen3_camera_color_frame"};
    tf::Stamped<tf::Vector3> velVectorToolFrameOut;

    listener.transformVector("kinova_gen3_pencilTipFrame", velVectorCameraFrame, velVectorToolFrameOut);

    tf::Stamped<tf::Vector3> velVectorToolFrame = {{velVectorToolFrameOut.getX(), velVectorToolFrameOut.getY(), velVectorToolFrameOut.getZ()}, ros::Time(0), "kinova_gen3_pencilTipFrame"};
    tf::Stamped<tf::Vector3> velVectorBaseFrame;

    listener.transformVector("kinova_gen3_base_link", velVectorToolFrame, velVectorBaseFrame);

    tf::Stamped<tf::Vector3> velVectorInAngular = {{velocityObjectFrame[3], velocityObjectFrame[4], velocityObjectFrame[5]}, ros::Time(0), "kinova_gen3_camera_color_frame"};
    tf::Stamped<tf::Vector3> velVectorOutAngular;

    listener.transformVector("kinova_gen3_pencilTipFrame", velVectorInAngular, velVectorOutAngular);
    std::cout <<"\n velocity wrt tool frame" <<velVectorOutAngular.getX() << " "<<velVectorOutAngular.getY()<<" "<<velVectorOutAngular.getZ()<< "\n";

    kinova_twist::TwistCommand kinovaVelocity;
    kinovaVelocity.reference_frame = 0;
    kinovaVelocity.twist.linear_x = velVectorBaseFrame.getX();
    kinovaVelocity.twist.linear_y = velVectorBaseFrame.getY();
    kinovaVelocity.twist.linear_z = velVectorBaseFrame.getZ();
    kinovaVelocity.twist.angular_x = velVectorOutAngular.getX();
    kinovaVelocity.twist.angular_y = velVectorOutAngular.getY();
    kinovaVelocity.twist.angular_z = velVectorOutAngular.getZ();
    velocityPublisher.publish(kinovaVelocity);
}

void PBVS::SendVelocityToGazebo(vpColVector v)
{
    //if running in simulator
    tf::Stamped<tf::Vector3> velVectorLinearCameraFrame {{v[0], v[1], v[2]}, ros::Time(0), "camera_sensor_link"};
    tf::Stamped<tf::Vector3> velVectorLinearToolFrameOut;
    listener.transformVector("camera_link", velVectorLinearCameraFrame, velVectorLinearToolFrameOut);
    
    tf::Stamped<tf::Vector3> velVectorAngularCameraFrame {{v[3], v[4], v[5]}, ros::Time(0), "camera_sensor_link"};
    tf::Stamped<tf::Vector3> velVectorAngularToolFrameOut;
    listener.transformVector("camera_link", velVectorAngularCameraFrame, velVectorAngularToolFrameOut);

    modelstate.twist.linear.x = velVectorLinearToolFrameOut.getX();
    modelstate.twist.linear.y = velVectorLinearToolFrameOut.getY();
    modelstate.twist.linear.z = velVectorLinearToolFrameOut.getZ();
    modelstate.twist.angular.x = velVectorAngularToolFrameOut.getX();
    modelstate.twist.angular.y = velVectorAngularToolFrameOut.getY();
    modelstate.twist.angular.z = velVectorAngularToolFrameOut.getZ();

    setmodelstate_cam.request.model_state = modelstate;
    client.call(setmodelstate_cam);
}