
#include <iostream>
#include <vector>

#include <kinova_twist/TwistCommand.h>

#include <ros/ros.h>
#include <ros/init.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>

#include <visp/vpTrackingException.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp/vpImage.h> 
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp_bridge/image.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/core/vpImage.h>

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/bind.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <ibvs_arm.h>


bool manualInitialisation = false;
constexpr float IMAGE_WIDTH  = 1280;
constexpr float IMAGE_HEIGHT = 720;
constexpr double FOCAL_LENGTH_X = 1289.555982;
constexpr double FOCAL_LENGTH_Y = 1289.100645;
constexpr double U_O = 618.98899;
constexpr double V_O = 223.8499;

using namespace cv::xfeatures2d;

IBVS::IBVS(ros::NodeHandle &nh) : dot(4),  detectedBorders(4), detectedPoints(4)
{
	SetParameters(nh);

	currentImage = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);  
	initialImage = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
	grayFrame = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);

	featurePoints.reserve(4);
	storedFeaturePoints.reserve(4);
	distanceBetweenPoints.reserve(4);
	previous_z_k.setZero();
	z_k_plus_1.setZero();

	if (initialImage.empty())
	{
        std::cout << "!!! Failed imread(): image not found" << std::endl;
		namedWindow("initial image", WINDOW_AUTOSIZE );
		cv::imshow("initial image", initialImage); 
	}

	listener.waitForTransform("kinova_gen3_base_link", "kinova_gen3_camera_color_frame", ros::Time(0), ros::Duration(1));
	image_transport::ImageTransport imageTransport(nh);
	rawImageSubscriber = imageTransport.subscribe("kinova/camera_gripper/color/image_raw", 1, &IBVS::RawImageCallback, this);
	velocityPublisher = nh.advertise<kinova_twist::TwistCommand>("/my_gen3/in/cartesian_velocity", 1);
	IsObjectDetectedPublisher = nh.advertise<std_msgs::Bool>("/is_object_detected_ibvs", 1);
	distanceSubscriber = nh.subscribe("/my_gen3/in/distanceToObject", 1, &IBVS::IsDistanceGreaterThanThreshold, this);
}

void IBVS::SetParameters(ros::NodeHandle& n)
{
    n.getParam("/VisualServoing/parameters/servo_type", servo_type);
    n.getParam("/VisualServoing/parameters/task", operation);
    n.getParam("/VisualServoing/parameters/touch_distance", touch_distance);
    n.getParam("/VisualServoing/parameters/inspection_distance", inspection_distance);
	n.getParam("/VisualServoing/parameters/adaptive_gain_zero_ibvs", adaptive_gain_zero);
	n.getParam("/VisualServoing/parameters/adaptive_gain_inf_ibvs", adaptive_gain_inf);
	n.getParam("/VisualServoing/parameters/error_threshold", errorThreshold);
	n.getParam("/VisualServoing/parameters/tracking_error_threshold", tracking_error_threshold);

	auto GetParam = [&](const std::string& operation)
	{
		n.getParam("/VisualServoing/parameters/IBVS/" + operation + "/desired_camera_transform_wrt_object_frame", desiredTransform);
		n.getParam("/VisualServoing/parameters/IBVS/" + operation + "/model_reference_points", reference_points);
		n.getParam("/VisualServoing/parameters/IBVS/" + operation + "/feature_pixel_values", pixel_values);
		n.getParam("/VisualServoing/parameters/IBVS/" + operation + "/initial_image", initialImageLocation);
	};

	GetParam(operation);

	if(servo_type == "PlanningBased" || servo_type == "PBVS" )
		ros::shutdown();

	if(servo_type == "Hybrid")
		isHybrid = true;

	initialImage = imread(initialImageLocation, CV_32F);

	if (reference_points.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		for (int i = 0; i < reference_points.size(); i++)
		{
			XmlRpc::XmlRpcValue referenceObject = reference_points[i];
			double val1 = reference_points[i][0];
			double val2 = reference_points[i][1];
			double val3 = reference_points[i][2];
			point.push_back(vpPoint(val1, val2, val3));
		}
	}
	
	if (pixel_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		for (int i = 0; i < pixel_values.size(); i++)
		{
			XmlRpc::XmlRpcValue pixelObject = pixel_values[i];
			int val1 = pixelObject[0];
			int val2 = pixelObject[1];
			storedFeaturePoints.push_back({(float)val1, (float)val2});
		}
	}

	distanceBetweenPoints.emplace_back(GetDistanceBetweenPoints(point[0], point[1]));
	distanceBetweenPoints.emplace_back(GetDistanceBetweenPoints(point[1], point[2]));
	distanceBetweenPoints.emplace_back(GetDistanceBetweenPoints(point[0], point[2]));
	distanceBetweenPoints.emplace_back(GetDistanceBetweenPoints(point[2], point[3]));
}

double GetDistanceBetweenPoints(vpPoint p1, vpPoint p2)
{
	return sqrt(pow(p1.get_X() - p2.get_X(), 2) + pow(p1.get_Y() - p2.get_Y(), 2));
}

void IBVS::IsDistanceGreaterThanThreshold(const std_msgs::Float64& distanceToObject)
{
	isDistanceLessThanThreshold = true;
}


void IBVS::DisplayWindow(vpImage<unsigned char> &I)
{
#if defined(VISP_HAVE_X11)
	vpDisplayX d(I, 0, 0, "Current camera view");
#elif defined(VISP_HAVE_GDI)
	vpDisplayGDI d(I, 0, 0, "Current camera view");
#elif defined(VISP_HAVE_OPENCV)
	vpDisplayOpenCV d(I, 0, 0, "Current camera view");
#else
	std::cout << "No image viewer is available..." << std::endl;
#endif
}

void IBVS::display_trajectory(const vpImage<unsigned char>& I, const std::vector<vpDot2>& dot)
{
	static std::vector<vpImagePoint> traj[4];
	for (unsigned int i = 0; i < 4; i++) {
		traj[i].push_back(dot[i].getCog());
	}
	for (unsigned int i = 0; i < 4; i++) {
		for (unsigned int j = 1; j < traj[i].size(); j++) {
			vpDisplay::displayLine(I, traj[i][j - 1], traj[i][j], vpColor::green);
		}
	}
}

void IBVS::ShutDownNodeAndSetKinovaVelocityToZero()
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


void IBVS::RawImageCallback(const sensor_msgs::ImageConstPtr& rawImage)
{
    auto image = cv_bridge::toCvCopy(rawImage, sensor_msgs::image_encodings::BGR8)->image;
    ibvsKeypointTracker.SetImageHeightAndWidth(IMAGE_HEIGHT, IMAGE_WIDTH);

    // Handle Hybrid Mode Object Detection
    if (isHybrid)
    {
        HandleHybridMode(image);
        return; // Exit early if distance condition isn't met in Hybrid mode
    }

    try 
    {
        static vpImage<unsigned char> I(IMAGE_HEIGHT, IMAGE_WIDTH, 255);
        static vpDisplayOpenCV d(I, 0, 0, "SURF tracking");
        
        vpImageConvert::convert(image, I);
        vpDisplay::display(I);
        vpDisplay::flush(I);

        // Camera parameters and desired transformation matrix
        vpHomogeneousMatrix cdMo(desiredTransform[0], desiredTransform[1], desiredTransform[2], 
                                  desiredTransform[3], desiredTransform[4], desiredTransform[5]);
        vpCameraParameters cameraParametersIntrinsic(FOCAL_LENGTH_X, FOCAL_LENGTH_Y, U_O, V_O);
        vpServoDisplay::display(task, cameraParametersIntrinsic, I, vpColor::green, vpColor::red);

        if (isInitialized)
        {
            InitializeFeatures(I);
            return; // Exit after initialization
        }

        ProcessImageTracking(I, image);

        ComputeControlLaw(I);

 
        if (IsErrorBelowThreshold())
        {
            ROS_INFO("Desired pose reached");
            ShutDownNodeAndSetKinovaVelocityToZero();
        }

        vpDisplay::flush(I);
    }
    catch(const vpException &e)
    {
        std::cout << e;
        ShutDownNodeAndSetKinovaVelocityToZero();
    }
}

void IBVS::HandleHybridMode(const cv::Mat& image)
{
    if (isDistanceLessThanThreshold)
    {
        if (!isObjectDetected)
        {
            isObjectDetected = ibvsKeypointTracker.IsObjectDetected(initialImage, image, storedFeaturePoints);
        }
        std_msgs::Bool msg;
        msg.data = isObjectDetected;
        IsObjectDetectedPublisher.publish(msg);
    }
}

// Initial feature selection and tracking
void IBVS::InitializeFeatures(vpImage<unsigned char>& I)
{
    vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
    if (manualInitialisation)
    {
        // Wait for user to select features
        do {
            vpDisplay::displayText(I, 10, 10, "Left click to select a point, right to start tracking", vpColor::red);
            vpImagePoint ip;
            if (vpDisplay::getClick(I, ip, button, false)) 
            {
                if (button == vpMouseButton::button1) 
                {
                    if (feature.size() < 4)
                        feature.push_back(cv::Point2f((float)ip.get_u(), (float)ip.get_v()));
                    else
                        detectedBorders.push_back(cv::Point2f((float)ip.get_u(), (float)ip.get_v()));
                    vpDisplay::displayCross(I, ip, 12, vpColor::green);
                }
            }
            vpDisplay::flush(I);
        } while (button != vpMouseButton::button3);
    }
    else
    {
        feature = storedFeaturePoints;
    }

    // Create features and initialize tracking
	// cdMo is the desired object pose w.r.t camera. This is calculated by perspective projection of the the 3D points w.r.t to 
	// desired camera pose 
    for (unsigned int i = 0; i < 4; i++)
    {
        vpImagePoint cog = vpImagePoint(feature[i].y, feature[i].x);
        cog = vpImagePoint(cog.get_v(), cog.get_u());
        point[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], point[i]);
        vpDisplay::flush(I);
        vpFeatureBuilder::create(p[i], cameraParametersIntrinsic, cog);
        task.addFeature(p[i], pd[i]);
    }

    cvtColor(initialImage, grayFrame, CV_BGR2GRAY);
    intialImage = grayFrame;
	isInitialized = true;
}

// Update feature points and compute the transformation based on homography
void IBVS::ProcessImageTracking(vpImage<unsigned char>& I, const cv::Mat& image)
{
    cvtColor(image, grayFrame, CV_BGR2GRAY);
    currentImage = grayFrame;

    // Calculate the transformation from the initial template to the current image
    auto scene_corners = ibvsKeypointTracker.CalculateTranformBasedOnHomography(currentImage, initialImage, feature);

    std::vector<vpImagePoint> cogVector;
    for (std::size_t i = 0; i < 4; i++) 
    {
        vpImagePoint cog = vpImagePoint(scene_corners[i].y, scene_corners[i].x);
        cog = vpImagePoint(cog.get_v(), cog.get_u());    
        vpFeatureBuilder::create(p[i], cameraParametersIntrinsic, cog);
        cogVector.push_back(cog);
    }

    CalculateDistance(cogVector);

    // Update Z values for the features
    for (std::size_t i = 0; i < 3; i++) 
    {
        p[i].set_Z(z_k_plus_1(i));
    }
    p[3].set_Z(z_k_plus_1(2)); 
}

// Control loop and velocity computation
void IBVS::ComputeControlLaw(vpImage<unsigned char>& I)
{
    vpAdaptiveGain adaptiveGain(adaptive_gain_zero, adaptive_gain_inf, 30);   // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
    vpColVector velocityCameraFrame;
    task.setLambda(adaptiveGain);
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::DESIRED);

    velocityCameraFrame = task.computeControlLaw();
    double error = (task.getError()).sumSquare(); // error = s^2 - s_star^2
    display_trajectory(I, dot);
    
    if (error < errorThreshold)
    {
        ROS_INFO("Desired pose reached");
        ShutDownNodeAndSetKinovaVelocityToZero();
    }

    SendVelocityToKinematicsNode(velocityCameraFrame, tf::Vector3{0,0,0});
}

bool IBVS::IsErrorBelowThreshold() const
{
    return (task.getError()).sumSquare() < errorThreshold;
}

// For image based visual servoing, the 3D distance of the feature points from the camera must be known. 
// Distance of each feature point from the camera has to be calculated.
// Implementation is based on the method in paper http://ieeexplore.ieee.org/document/8297195/
std::tuple<double, double, double, double> IBVS::CalculateDistance(const std::vector<vpImagePoint>& cogVector)
{
	auto GetCartesianPoints = [] (int i, int j) -> std::tuple<int, int, int, int>
	{
		double u1 = cogVector[i].get_i();
		double v1 = cogVector[i].get_j();
		double u2 = cogVector[j].get_i() ;
		double v2 = cogVector[j].get_j() ;
		return std::tuple<int, int, int, int>{u1, v1, u2, v2};
	};

	double l1 = distanceBetweenPoints[0];
	auto[u1, v1, u2, v2] = GetCartesianPoints(0,1);
	Equation equation1 = InitEquations(u1, v1, u2, v2, l1);

	double l2 = distanceBetweenPoints[1];
	[u1, v1, u2, v2]  = GetCartesianPoints(1,2);
	Equation equation2 = InitEquations(u1, v1, u2, v2, l2);

	double l3 = distanceBetweenPoints[2];
	[u1, v1, u2, v2]  = GetCartesianPoints(0,2);
	Equation equation3 = InitEquations(u1, v1, u2, v2, l3);

	std::vector<Equation> equations{equation1, equation2, equation3};

	auto [z0, z1, z2, z3] = CalculateDistanceGuesses(cogVector);
	Eigen::Vector3d z_k{z0, z1, z2};
	previous_z_k = {z0, z1, z2};
	NewtonRaphson(z_k, GetGZ(equations, z_k), equations);
}

std::tuple<double, double, double, double> IBVS::CalculateDistanceGuesses(const std::vector<vpImagePoint>& cogVector)
{
	//calculate distance estimate to be used as initial guess to newtonraphson method
	auto z0 = FOCAL_LENGTH_X * distanceBetweenPoints[0]/sqrt(pow(cogVector[0].get_i() - cogVector[1].get_i(),2) +
				pow(cogVector[0].get_j() - cogVector[1].get_j(),2)) ;

	auto z1 = FOCAL_LENGTH_Y * distanceBetweenPoints[1]/sqrt(pow(cogVector[1].get_i() - cogVector[2].get_i(),2) +
				pow(cogVector[1].get_j() - cogVector[2].get_j(),2)) ;

	//diagonal distance between point 1 and point 3
	auto z2 = FOCAL_LENGTH_X *  distanceBetweenPoints[2]/sqrt(pow(cogVector[0].get_i() - cogVector[2].get_i(),2) +
				pow(cogVector[0].get_j() - cogVector[2].get_j(),2));

	//this distance is calculated but not used since the distance calculation method works better with only three points
	auto z3 = FOCAL_LENGTH_Y * distanceBetweenPoints[3]/sqrt(pow(cogVector[0].get_i() - cogVector[3].get_i(),2) +
				pow(cogVector[0].get_j() - cogVector[3].get_j(),2));
	return std::make_tuple(z0, z1, z2, z3);
}

void IBVS::NewtonRaphson(Eigen::Vector3d& z_k, Eigen::Vector3d& GZ, std::vector<Equation>& equations)
{
	Eigen::Matrix3d jacobianGZ;
	for(int i = 0; i < 1000, i++)
	{
		jacobianGZ<<(2*equations[0].A*z_k(0) -  equations[0].C * z_k(1)), (2*equations[0].B*z_k(1) -  equations[0].C * z_k(0)),                                                 0,   
					0,2*equations[1].A*z_k(1) - equations[1].C * z_k(2), 2*equations[1].B*z_k(2) - equations[1].C * z_k(1),                                                  

					2*equations[2].A*z_k(0) -  equations[2].C * z_k(2),                                                    0,                                                   2*equations[2].B*z_k(2)- equations[2].C * z_k(0);

		z_k_plus_1 = z_k - jacobianGZ.colPivHouseholderQr().solve(GZ);;
		auto error = previous_z_k - z_k_plus_1;
		double RMSE = sqrt(pow(error[0], 2) + pow(error[1], 2) + pow(error[2], 2));
		if(RMSE < 0.0001)
			return ;
		previous_z_k = z_k_plus_1;
	}
}

Equation& IBVS::InitEquations(double u1, double v1, double u2, double v2, double l)
{
	Equation equation;
	u1 = u1 - U_O;
	u2 = u2 - U_O;
	v1 = v1 - V_O;
	v2 = v2 - V_O;
	equation.A = pow(u1,2) + pow(v1,2) + pow(FOCAL_LENGTH_X,2);
	equation.B = pow(u2,2) + pow(v2,2) + pow(FOCAL_LENGTH_X,2);
	equation.C = 2*(u1*u2 + v1*v2 + pow(FOCAL_LENGTH_X, 2));
	equation.D = pow(FOCAL_LENGTH_X,2) * pow(l,2);
	return equation;
}

Eigen::Vector3d IBVS::GetGZ(std::vector<Equation> equations, Eigen::Vector3d z)
{
	Eigen::Vector3d GZ;
	GZ(0) = (equations[0].A * pow(z(0),2) + equations[0].B * pow(z(1),2) - equations[0].C* z(0)*z(1) - equations[0].D);
	GZ(1) = (equations[1].A * pow(z(1),2) + equations[1].B * pow(z(2),2) - equations[1].C* z(1)*z(2) - equations[1].D);
	GZ(2) = (equations[2].A * pow(z(0),2) + equations[2].B * pow(z(2),2) - equations[2].C* z(0)*z(2) - equations[2].D);
	return GZ;
}

void IBVS::SendVelocityToKinematicsNode(vpColVector velocityCameraFrame, tf::Vector3 pointTranslation)
{
	geometry_msgs::Twist velocity;
	tf::Vector3 out_rot;
	tf::Vector3 out_vel;
	geometry_msgs::Twist interframe_twist;
	tf::StampedTransform transform;

	// velocity is calculated in camera frame which is converted to end-effector frame (tool-frame)
	// for kinova gen3 arm, cartesian velocity has to send to the hardware where linear velocity must be w.r.t end-effector 
	// and angular velocity w.r.t base link.
	tf::Stamped<tf::Vector3> velVectorCameraFrame {{velocityCameraFrame[0], velocityCameraFrame[1], velocityCameraFrame[2]}, ros::Time(0), "kinova_gen3_camera_color_frame"};
	tf::Stamped<tf::Vector3> velVectorToolFrameOut;

	listener.transformVector("kinova_gen3_tool_frame", velVectorCameraFrame, velVectorToolFrameOut);

	tf::Stamped<tf::Vector3> velVectorToolFrame = {{velVectorToolFrameOut.getX(), velVectorToolFrameOut.getY(), velVectorToolFrameOut.getZ()}, ros::Time(0), "kinova_gen3_tool_frame"};
	tf::Stamped<tf::Vector3> velVectorBaseFrame;

	listener.transformVector("kinova_gen3_base_link", velVectorToolFrame, velVectorBaseFrame);

	tf::Stamped<tf::Vector3> velVectorInAngular = {{velocityCameraFrame[3], velocityCameraFrame[4], velocityCameraFrame[5]}, ros::Time(0), "kinova_gen3_camera_color_frame"};
	tf::Stamped<tf::Vector3> velVectorOutAngular;

	listener.transformVector("kinova_gen3_tool_frame", velVectorInAngular, velVectorOutAngular);
	
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
