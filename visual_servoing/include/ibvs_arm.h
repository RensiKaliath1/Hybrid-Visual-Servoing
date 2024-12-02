#include <ros/ros.h>
#include <ros/init.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>

#include <std_msgs/Float64.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>
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


#include <image_transport/image_transport.h>

#include <ibvs_keypoint_tracker.h>

struct Equation
{
	double A;
	double B;
	double C;
	double D;
};

class IBVS
{
        vpServo task;
        vpKltOpencv tracker;
		vpFeaturePoint p[4], pd[4];
		std::vector<vpDot2> dot;
		std::vector<vpPoint> point;
		std::vector<vpImagePoint> cogVector;
        cv::Mat depthImage;
        cv::Mat startImage;
        vpDisplayOpenCV d;
        std::vector<cv::Point2f> feature;
		std::vector<cv::Point2f> scene_corners;
		Eigen::Vector3d previous_z_k;
		std::string servo_type;
		std::string operation;
		double touch_distance;
		double inspection_distance;
		XmlRpc::XmlRpcValue desiredTransformXmlRpc;
		std::vector<double> desiredTransform;
		std::string modelParametersFilePath;
		XmlRpc::XmlRpcValue reference_points;
		XmlRpc::XmlRpcValue pixel_values;
		double adaptive_gain_zero = 0;
		double adaptive_gain_inf = 0;
		double tracking_error_threshold = 0;
		double previous_error;
		IBVSKeypointTracker ibvsKeypointTracker;
		double errorThreshold;
        tf::Transformer transformer;
        tf::TransformListener listener;
        ros::Time start;
        ros::Time end;
        cv::Mat currentImage, previousImage;
 	    cv::Mat img_matches;
		std::vector<cv::Point2f> borders;
        bool isObjectDetected = false;
		bool isDistanceLessThanThreshold = false;
		int objectDetectedIndex = 0;
		int counter = 0;
        cv::Mat initialImage;
		std::string initialImageLocation;
        std::vector<cv::Point2f> detectedBorders;
	    std::vector<cv::Point2f> detectedPoints;
		Eigen::Vector3d z_k_plus_1;
		std::vector<cv::Point2f> storedFeaturePoints;
		std::vector<cv::Point2f> storedBorders;
		std::vector<double> distanceBetweenPoints;
        cv::Mat previousFixedImage;
		bool isHybrid;
		bool isInitialized = false;
		bool done = true;
    public:
		IBVS(ros::NodeHandle& nh);
		void SetParameters(ros::NodeHandle&);
		void ShutDownNodeAndSetKinovaVelocityToZero();
		void NewtonRaphson(Eigen::Vector3d& z_k, Eigen::Vector3d& GZ, std::vector<Equation>& equations);
        Equation& InitEquations(double u1, double v1, double u2, double v2, double l);
		Eigen::Vector3d GetGZ(std::vector<Equation> equations, Eigen::Vector3d z);
		void IsDistanceGreaterThanThreshold(const std_msgs::Float64& distanceToObject);
		std::tuple<double, double, double, double> CalculateDistanceGuesses(const std::vector<vpImagePoint>& cogVector);       
		std::tuple<double, double, double, double> CalculateDistance(const std::vector<vpImagePoint>& cogVector); 
        
		void SendVelocityToKinematicsNode(vpColVector velocityObjectFrame, tf::Vector3 pointTranslation);
        void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot);
        void DisplayWindow(vpImage<unsigned char>&  I);

		void InitializeFeatures(vpImage<unsigned char>& I)
        void RawImageCallback(const sensor_msgs::ImageConstPtr& rawImage);
		void HandleHybridMode(const cv::Mat& image);
		void ProcessImageTracking(vpImage<unsigned char>& I, const cv::Mat& image);
		void ComputeControlLaw(vpImage<unsigned char>& I);
		bool IsErrorBelowThreshold() const;
		void ProcessImageTracking(vpImage<unsigned char>& I, const cv::Mat& image);

        image_transport::Subscriber rawImageSubscriber;
		ros::Subscriber distanceSubscriber;
        ros::Publisher velocityPublisher;
		ros::Publisher IsObjectDetectedPublisher;
};
