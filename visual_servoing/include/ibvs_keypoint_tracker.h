#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <std_msgs/Float64.h>
class IBVSKeypointTracker
{
	cv::Mat img_matches;
	std::vector<cv::Point2f> storedBorders;
	double IMAGE_HEIGHT, IMAGE_WIDTH;
    std::vector<cv::Point2f> detectedBorders;
	std::vector<cv::Point2f> detectedPoints;
	cv::Mat previousFixedImage;
	public: 
		void SetImageHeightAndWidth(double imageHeight, double imageWidth);
		std::tuple<cv::Point2f, cv::Point2f> GetBoundingBox(std::vector<cv::Point2f> imageBorders);
        bool IsObjectDetected(const cv::Mat& fixedCameraImage, const cv::Mat& armCameraImage, std::vector<cv::Point2f>storedFeaturePoints);
		void IsDistanceGreaterThanThreshold(const std_msgs::Float64& distanceToObject);
	    cv::Mat GetROIImage(const cv::Mat& croppedImage, const cv::Point2f& boundingBoxMin, const cv::Point2f& boundingBoxMax);
        std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>, std::vector<cv::DMatch>,  std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> GetKeypointMatches(const cv::Mat& currentImage, const cv::Mat& previousImage);
	 	std::vector<cv::Point2f> CalculateTranformBasedOnHomography(const cv::Mat& currentIMage, const cv::Mat& previousImage, const std::vector<cv::Point2f>& feature);
	 	cv::Mat GetCroppedImage(cv::Mat image, cv::Point2f boundingBoxMin, cv::Point2f boundingBoxMax);
};
