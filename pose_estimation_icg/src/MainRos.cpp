/** Written by the Object Handling subgroup
 **/

#include "ros/init.h"
#include "std_msgs/Bool.h"
#include <icg/MainRos.h>

/** Initialization class **/
RosICGTracker::RosICGTracker (ros::NodeHandle *nodehandle)
{
    // Initialization is actually done in the camera info callback function after the camera parameters arrive
 
    nh = nodehandle;
    nh->param("/VisualServoing/parameters/servo_type", servo_type);
    nh->param("/VisualServoing/parameters/task", task);
    nh->param("/VisualServoing/parameters/initial_approach", initial_approach);
    if(servo_type == "IBVS")
        ros::shutdown();
    body_names_.emplace_back("asymmetric_pipestar_without_cap");
    // Read arguments from launch file
   // nh->param ("/VisualServoing/parameters/ICG/bodies", body_names_, std::vector<std::string>());
    nh->param ("/VisualServoing/parameters/ICG/useDepth", useDepthCamera);
    nh->param ("/VisualServoing/parameters/ICG/manualDetectorFilename", manualDetectorFilename);
    icgShutdownSubscriber = nh->subscribe("/shutdownICG",5, &RosICGTracker::ShutdownNode, this);
    // Subscribe to rgb camera info
    if(initial_approach == "PlanningBased" && servo_type != "PBVS")
        cameraTopic = "/camera/rgb";
    else
        cameraTopic = "/kinova/camera_gripper/color";
    rgbCameraInfoSub = nh->subscribe (cameraTopic + "/camera_info", 10, &RosICGTracker::rgbCameraInfoCallback, this);
    ROS_INFO ("Waiting for RGB camera info.\n");

    // Subscribe to depth camera info
    if (useDepthCamera)
    {
        depthCameraInfoSub = nh->subscribe ("/kinova/camera_gripper/depth/camera_info", 10, &RosICGTracker::depthCameraInfoCallback, this);
        ROS_INFO ("Waiting for depth camera info.\n");
    }
    // Advertise the estimated pose as a publisher
    posePub = nh->advertise<estimated_pose_msg::estimatedPose> ("tracked_object_poses", 100);
}
void RosICGTracker::ShutdownNode(const std_msgs::Bool shutdown)
{
    if(shutdown.data)
        ros::shutdown();
}
int RosICGTracker::initICG()
{
    initDone = true;
    //	std::cout << "Debug Mode: " << cvv::debugMode() << std::endl;
    std::string packagePath = ros::package::getPath ("pose_estimation_icg");
    std::string srcPath = packagePath.substr (0, packagePath.size() - 20);
    std::string saveImagesPath = srcPath + "/recordings";
    std::string bodyPath = srcPath + "/resources" + "/models";
    const std::filesystem::path directory{bodyPath};

    const bool kUseDepthViewer = useDepthCamera;
    constexpr bool kMeasureOcclusions = true;
    constexpr bool kModelOcclusions = false;
    constexpr bool kVisualizePoseResult = true;
    constexpr bool kSaveImages = false;
    const std::filesystem::path save_directory{saveImagesPath};

    // Set up tracker and renderer geometry
    tracker_ptr = std::make_shared<icg::Tracker> ("tracker");
    auto renderer_geometry_ptr{
        std::make_shared<icg::RendererGeometry> ("renderer geometry")};

    // Set up publisher class
    publisher_ptr = std::make_shared<icg::RosPublisher> ("pose");
    tracker_ptr->AddPublisher (publisher_ptr);
    publisher_ptr->AddRosPublisher (posePub);
    // Pose data gets calculated relative to the publisher camera
    publisher_ptr->AddCamera (color_camera_ptr);

    // Set up viewers
    auto color_viewer_ptr{std::make_shared<icg::NormalColorViewer> (
        "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
    if (kSaveImages)
        color_viewer_ptr->StartSavingImages (save_directory, "bmp");
    tracker_ptr->AddViewer (color_viewer_ptr);
    if (kUseDepthViewer)
    {
        auto depth_viewer_ptr{std::make_shared<icg::NormalDepthViewer> (
            "depth_viewer", depth_camera_ptr, renderer_geometry_ptr, 0.3f, 1.0f)};
        if (kSaveImages)
            depth_viewer_ptr->StartSavingImages (save_directory, "bmp");
        tracker_ptr->AddViewer (depth_viewer_ptr);
    }

    // Set up depth renderer
    auto color_depth_renderer_ptr{
        std::make_shared<icg::FocusedBasicDepthRenderer> (
            "color_depth_renderer", renderer_geometry_ptr, color_camera_ptr)};
    std::shared_ptr<icg::FocusedBasicDepthRenderer> depth_depth_renderer_ptr;
    if (useDepthCamera)
    {
        depth_depth_renderer_ptr =
            std::make_shared<icg::FocusedBasicDepthRenderer> (
                "depth_depth_renderer", renderer_geometry_ptr, depth_camera_ptr);
    }
    if (body_names_.empty())
    {
        ROS_INFO ("No body was specified, thus neither detection nor tracking can start\n");
    }
    for (const auto body_name : body_names_)
    {
        std::cout << "Processing body:" << body_name << std::endl;
        // Set up body
        std::filesystem::path metafile_path{directory / body_name / (body_name + ".yaml")};
        std::filesystem::exists (metafile_path);
        auto body_ptr{std::make_shared<icg::Body> (body_name, metafile_path)};
        renderer_geometry_ptr->AddBody (body_ptr);
        color_depth_renderer_ptr->AddReferencedBody (body_ptr);
        if (useDepthCamera)
        {
            depth_depth_renderer_ptr->AddReferencedBody (body_ptr);
        }
        publisher_ptr->AddReferencedBody (body_ptr);

        // Set up detector
        std::filesystem::path detector_path{directory / body_name /
                                            (body_name + "_manual_detector.yaml")};

        auto detector_ptr{std::make_shared<icg::ManualDetector> (
            body_name + "_detector", detector_path, body_ptr, color_camera_ptr)};
        tracker_ptr->AddDetector (detector_ptr);

        // Set up models
        auto region_model_ptr{std::make_shared<icg::RegionModel> (
            body_name + "_region_model", body_ptr,
            directory / body_name / (body_name + "_region_model.bin"))};
        std::shared_ptr<icg::DepthModel> depth_model_ptr;
        if (useDepthCamera)
        {
            depth_model_ptr = std::make_shared<icg::DepthModel> (
                body_name + "_depth_model", body_ptr,
                directory / body_name / (body_name + "_depth_model.bin"));
        }
        // Set up modalities
        auto region_modality_ptr{std::make_shared<icg::RegionModality> (
            body_name + "_region_modality", body_ptr, color_camera_ptr,
            region_model_ptr)};
        std::shared_ptr<icg::DepthModality> depth_modality_ptr;
        if (useDepthCamera)
        {
            depth_modality_ptr = std::make_shared<icg::DepthModality> (
                body_name + "_depth_modality", body_ptr, depth_camera_ptr,
                depth_model_ptr);
        }
        if (kVisualizePoseResult)
        {
            region_modality_ptr->set_visualize_pose_result (true);
        }
        if (kMeasureOcclusions & useDepthCamera)
        {
            region_modality_ptr->MeasureOcclusions (depth_camera_ptr);
            depth_modality_ptr->MeasureOcclusions();
        }
        if (kModelOcclusions & useDepthCamera)
        {
            region_modality_ptr->ModelOcclusions (color_depth_renderer_ptr);
            depth_modality_ptr->ModelOcclusions (depth_depth_renderer_ptr);
        }

        // Set up optimizer
        auto body1_optimizer_ptr{
            std::make_shared<icg::Optimizer> (body_name + "_optimizer")};
        body1_optimizer_ptr->AddModality (region_modality_ptr);
        if (useDepthCamera)
        {
            body1_optimizer_ptr->AddModality (depth_modality_ptr);
        }
        tracker_ptr->AddOptimizer (body1_optimizer_ptr);
    }

    // Subscribe to GETjag camera
    image_transport::ImageTransport it (*nh);
    imageSubRgb = it.subscribe (cameraTopic + "/image_raw", 10, &RosICGTracker::rgbImageCallback, this);
    if (useDepthCamera)
    {
        imageSubDepth = it.subscribe ("/kinova/camera_gripper/depth/image_raw", 10, &RosICGTracker::depthImageCallback, this);
    }
    ROS_INFO ("Initialization complete.\n");

    return 0;
}

int RosICGTracker::runICG()
{
    runICGDone = true;
    if (!tracker_ptr->SetUp())
    {
        return 0;
    }
    icgThread = std::thread (&icg::Tracker::RunTrackerProcess, tracker_ptr, false, false);
    return 0;
}

void RosICGTracker::rgbCameraInfoCallback (const sensor_msgs::CameraInfoConstPtr cameraInfo)
{
    // Currently, only initialize the PnP detector once the first camera parameter info arrives
    if (firstInitRgb == false)
    {
        rgbCameraInfoSub.shutdown();
        ROS_INFO ("RGB Camera parameters arrived. Unsubscribing from rgb camera info \n");

        // Set up camera
        color_camera_ptr =
            std::make_shared<icg::RosCameraColorCamera> ("ros_camera_color", cameraInfo);

        firstInitRgb = true;

        // Initialize the rest if both rgb and depth cameras were initialized
        if (firstInitRgb && (firstInitDepth || !useDepthCamera) && !initDone)
        {
            initICG();
        }
    }
}

void RosICGTracker::depthCameraInfoCallback (const sensor_msgs::CameraInfoConstPtr cameraInfo)
{
    // Currently, only initialize the PnP detector once the first camera parameter info arrives
    if (firstInitDepth == false)
    {
        depthCameraInfoSub.shutdown();
        ROS_INFO ("Depth camera parameters arrived. Unsubscribing from depth camera info \n");

        // Set up camera
        depth_camera_ptr =
            std::make_shared<icg::RosCameraDepthCamera> ("ros_camera_depth", cameraInfo);

        firstInitDepth = true;

        // Initialize the rest if both rgb and depth cameras were initialized
        if (firstInitRgb && firstInitDepth && !initDone)
        {
            initICG();
        }
    }
}

void RosICGTracker::rgbImageCallback (const sensor_msgs::ImageConstPtr &msg)
{
    if (firstInitRgb == false)
    {
        // Camera parameters have not arrived to the cameraInfoCallback function yet
        return;
    }
    firstImageRgb = true;
    // std::cout << "rgb image arrived" << std::endl;
    // ROS uses callbacks for new images while ICG uses active polling.
    // To make them compatible, the callback function has to fake a buffer for new images.
    cv_ptr_rgb = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);

    // Convert ros image to opencv image
    cv::Mat frame;
    frame = cv_ptr_rgb->image;
    // Add the message header to the publisher object so it can get published
    publisher_ptr->msg_header = msg->header;
    color_camera_ptr->UpdateCapture (frame);

    // Run ICG if the first RGB and depth images arrived (if depth camera is used)
    if (firstImageRgb && (firstImageDepth || !useDepthCamera) && !runICGDone)
    {
        runICG();
    }
}

void RosICGTracker::depthImageCallback (const sensor_msgs::ImageConstPtr &msg)
{
    if (firstInitDepth == false)
    {
        // Camera parameters have not arrived to the cameraInfoCallback function yet
        return;
    }
    firstImageDepth = true;
    // std::cout << "depth image arrived" << std::endl;
    // ROS uses callbacks for new images while ICG uses active polling.
    // To make them compatible, the callback function has to fake a buffer for new images.
    cv_ptr_depth = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::TYPE_16UC1);

    // Convert ros image to opencv image
    cv::Mat frame;
    frame = cv_ptr_depth->image;
    depth_camera_ptr->UpdateCapture (frame);

    // Run ICG if the first RGB and depth images arrived
    if (firstImageRgb && firstImageDepth && !runICGDone)
    {
        runICG();
    }
}
