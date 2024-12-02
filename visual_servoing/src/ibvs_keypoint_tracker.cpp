#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <std_msgs/Bool.h>

#include "visp3/core/vpImage.h"

#include "ibvs_keypoint_tracker.h"

using namespace cv::xfeatures2d;
using namespace cv;

void IBVSKeypointTracker::SetImageHeightAndWidth(double imageHeight, double imageWidth)
{
	IMAGE_HEIGHT = imageHeight;
	IMAGE_WIDTH = imageWidth;
}

std::tuple<std::vector<Point2f>, std::vector<Point2f>, std::vector<DMatch>, std::vector<KeyPoint>, std::vector<KeyPoint>> IBVSKeypointTracker::GetKeypointMatches(const cv::Mat& currentImage, const cv::Mat& previousImage)
{
	cv::Mat mask;
	int minHessian = 400;
	static Ptr<SURF> detector = SURF::create(minHessian);
	std::vector<KeyPoint> keypoints_object, keypoints_scene;
	cv::Mat descriptors_object;
	cv::Mat descriptors_scene;
	detector->detect(previousImage, keypoints_object, noArray());
	detector->detect(currentImage, keypoints_scene, noArray());
	
	detector->compute(previousImage, keypoints_object,descriptors_object);
	detector->compute(currentImage, keypoints_scene, descriptors_scene);

	//-- Step 2: Matching descriptor vectors with a FLANN based matcher
	// Since SURF is a floating-point descriptor NORM_L2 is used
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
	std::vector< std::vector<DMatch> > knn_matches;
	//-- Localize the object
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;
	std::vector<DMatch> good_matches;
	if(keypoints_object.size() >= 20 && keypoints_scene.size() >= 20) 
		matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
	else
	 	return {obj, scene, good_matches, keypoints_object, keypoints_scene};
	//-- Filter matches using the Lowe's ratio test
	const float ratio_thresh = 0.75f;

	for (size_t i = 0; i < knn_matches.size(); i++)
	{
		if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
		{
			good_matches.push_back(knn_matches[i][0]);
		}
	}

	drawMatches( previousImage, keypoints_object, currentImage, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
	Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	for( size_t i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
		scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
	}

	return {obj, scene, good_matches, keypoints_object ,keypoints_scene};
}

std::tuple<cv::Point2f, cv::Point2f> IBVSKeypointTracker::GetBoundingBox(std::vector<cv::Point2f> imageBorders)
{
	double minX = 10000, minY = 10000, maxX = 0, maxY = 0;

	struct BoundingBox
	{
		cv::Point2f min;
		cv::Point2f max;
	} boundingBox;
	
	for(size_t i = 0; i< 4; i++)
	{
		auto value = imageBorders[i];
		minX = std::min((double)value.x, minX);
		minY = std::min((double)value.y, minY);

		maxX = std::max((double)value.x, maxX);
		maxY = std::max((double)value.y, maxY);

		boundingBox.min = cv::Point2f(minX,minY);
		boundingBox.max = cv::Point2f(maxX,maxY);
	}
	return {boundingBox.min, boundingBox.max};
}

cv::Mat IBVSKeypointTracker::GetROIImage(const cv::Mat& croppedImage, const cv::Point2f& boundingBoxMin, const cv::Point2f& boundingBoxMax)
{
	cv::Rect roi;
	roi.width = std::abs(boundingBoxMax.x - boundingBoxMin.x);
	roi.x =  boundingBoxMin.x;
	roi.height = std::abs(boundingBoxMax.y - boundingBoxMin.y);
	roi.y =  boundingBoxMin.y;
								
	cv::Mat square = cv::Mat::zeros( IMAGE_HEIGHT, IMAGE_WIDTH, croppedImage.type() );   
	cv::resize(croppedImage, square(roi), roi.size());
	return square;
}

cv::Mat IBVSKeypointTracker::GetCroppedImage(cv::Mat image, cv::Point2f boundingBoxMin, cv::Point2f boundingBoxMax)
{
	return image(cv::Rect(boundingBoxMin.x,boundingBoxMin.y, std::abs(boundingBoxMax.x - boundingBoxMin.x), std::abs(boundingBoxMax.y - boundingBoxMin.y)));
}

bool IBVSKeypointTracker::IsObjectDetected(const cv::Mat& initialImage, const cv::Mat& armCameraImage, std::vector<cv::Point2f>storedFeaturePoints)
{
	try
	{
		float imageHeight = initialImage.size().height;
		float imageWidth = initialImage.size().width;
		
		cv::Point2f boundingBoxMin{0,0};
		cv::Point2f boundingBoxMax{imageWidth, imageHeight};
		storedBorders = {{0,0}, {imageWidth, 0}, {imageWidth, imageHeight}, {0, imageHeight}};
		cv::Mat grayFrame;

		cvtColor(initialImage, grayFrame, CV_BGR2GRAY);

		namedWindow( "initial image", WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow("initial image", grayFrame); 
		cv::Mat currentImage;
		cvtColor(armCameraImage, currentImage, CV_BGR2GRAY);

		const auto [srcPoints, dstPoints, good_matches, keypoint_object, keypoint_scene] = GetKeypointMatches(currentImage, grayFrame);
		
		if(good_matches.size() > 100)
		{
			std::vector<char> mask;
			cv::Mat H = findHomography( srcPoints, dstPoints, RANSAC, 3, mask);
			int matchesAfterRANSAC = cv::countNonZero (mask);
			if(matchesAfterRANSAC > 50)
			{	
				perspectiveTransform(storedFeaturePoints, detectedPoints, H);
				perspectiveTransform(storedBorders, detectedBorders, H);
			
				for(size_t i = 0; i< detectedBorders.size(); i++)
				{
					//if((detectedBorders[i].x >IMAGE_WIDTH || detectedBorders[i].y >IMAGE_HEIGHT || detectedBorders[i].x < 0 || detectedBorders[i].y <0))//||
					//( detectedBorders[1].y < detectedPoints[1].y-5 ))
						//return false;
				}
				
				//-- Draw lines between the corners (the mapped object in the scene - image_2 )
				line( img_matches, detectedPoints[0] + Point2f((float)previousFixedImage.cols, 0),
				detectedPoints[1] + Point2f((float)previousFixedImage.cols, 0), Scalar(0, 255, 0), 4 );
				line( img_matches, detectedPoints[1] + Point2f((float)previousFixedImage.cols, 0),
				detectedPoints[2] + Point2f((float)previousFixedImage.cols, 0), Scalar( 0, 255, 0), 4 );
				line( img_matches, detectedPoints[2] + Point2f((float)previousFixedImage.cols, 0),
				detectedPoints[3] + Point2f((float)previousFixedImage.cols, 0), Scalar( 0, 255, 0), 4 );
				line( img_matches, detectedPoints[3] + Point2f((float)previousFixedImage.cols, 0),
				detectedPoints[0] + Point2f((float)previousFixedImage.cols, 0), Scalar( 0, 255, 0), 4 );
				//-- Show detected matches
				//imshow("Good Matches & Object detection", img_matches );
				return true;
			}
			else
				return false;
		}
	}
	catch(cv::Exception& e)
	{
		std::cout<<e.what()<<"Error occured";
		cv::destroyAllWindows();
	}
	return false;
}

std::vector<Point2f> IBVSKeypointTracker::CalculateTranformBasedOnHomography(const cv::Mat& currentImage, const cv::Mat& previousImage, const std::vector<cv::Point2f>& feature)
{
	const auto [srcPoints, dstPoints, good_matches, keypoint_object, keypoint_scene] = GetKeypointMatches(currentImage, previousImage);
	std::vector<char> mask;
	cv::Mat H = findHomography( srcPoints, dstPoints, RANSAC, 3, mask);
	
	drawMatches( previousImage, keypoint_object, currentImage, keypoint_scene, good_matches, img_matches, Scalar::all(-1),
	Scalar::all(-1), mask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	imshow("matches ", img_matches);
	std::vector<Point2f> scene_corners(4);
	perspectiveTransform(feature, scene_corners, H);
			//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	line( img_matches, scene_corners[0] + Point2f((float)previousImage.cols, 0),
	scene_corners[1] + Point2f((float)previousImage.cols, 0), Scalar(0, 255, 0), 4 );
	line( img_matches, scene_corners[1] + Point2f((float)previousImage.cols, 0),
	scene_corners[2] + Point2f((float)previousImage.cols, 0), Scalar( 0, 255, 0), 4 );
	line( img_matches, scene_corners[2] + Point2f((float)previousImage.cols, 0),
	scene_corners[3] + Point2f((float)previousImage.cols, 0), Scalar( 0, 255, 0), 4 );
	line( img_matches, scene_corners[3] + Point2f((float)previousImage.cols, 0),
	scene_corners[0] + Point2f((float)previousImage.cols, 0), Scalar( 0, 255, 0), 4 );
	//-- Show detected matches
	imshow("Good Matches & Object detection", img_matches );
    return scene_corners;
}