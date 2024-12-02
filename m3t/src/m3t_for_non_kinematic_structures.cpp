// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/m3t_on_sequence_ros.h>
/** Initialization class **/
Rosm3tTracker::Rosm3tTracker (ros::NodeHandle *nodehandle)
{
    // Initialization is actually done in the camera info callback function after the camera parameters arrive
    nh = nodehandle;

    // Read arguments from launch file
    nh = nodehandle;
    nh->param("/VisualServoing/parameters/servo_type", servo_type);
    nh->param("/VisualServoing/parameters/task", task);
    nh->param("/VisualServoing/parameters/initial_approach", initial_approach);
    if(servo_type == "IBVS")
        ros::shutdown();
    body_names_.emplace_back("asymmetric_pipestar_without_cap");
    // Read arguments from launch file
   // nh->param ("/VisualServoing/parameters/ICG/bodies", body_names_, std::vector<std::string>());
    nh->param ("/VisualServoing/parameters/M3T/useDepth", useDepthCamera);
    nh->param ("/VisualServoing/parameters/M3T/manualDetectorFilename", manualDetectorFilename);
    icgShutdownSubscriber = nh->subscribe("/shutdownICG",5, &Rosm3tTracker::ShutdownNode, this);
    // Subscribe to rgb camera info
    if(initial_approach == "PlanningBased" && servo_type != "PBVS")
        cameraTopic = "/camera/rgb";
    else
        cameraTopic = "/kinova/camera_gripper/color";
    cameraTopic = "/kinova/camera_gripper/color";
    rgbCameraInfoSub = nh->subscribe (cameraTopic + "/camera_info", 10, &Rosm3tTracker::rgbCameraInfoCallback, this);
    ROS_INFO ("Waiting for RGB camera info.\n");
    body_names_ = {"asymmetric_pipestar_without_cap"};
    useDepthCamera = false;
    // Subscribe to GETjag rgb and depth camera parameters
    image_transport::ImageTransport it (*nh);
    // Subscribe to rgb camera info
   // rgbCameraInfoSub = nh->subscribe ("/CameraRight/depth_camera/color/camera_info", 10, &Rosm3tTracker::rgbCameraInfoCallback, this);
    ROS_INFO ("Waiting for RGB camera info.\n");

    // Subscribe to depth camera info
    if (useDepthCamera)
    {
        depthCameraInfoSub = nh->subscribe ("/depth_camera/depth/camera_info", 10, &Rosm3tTracker::depthCameraInfoCallback, this);
        ROS_INFO ("Waiting for depth camera info.\n");
    }
    // Advertise the estimated pose as a publisher
    posePub = nh->advertise<estimated_pose_msg::estimatedPose> ("tracked_object_poses", 100);
}

void Rosm3tTracker::ShutdownNode(const std_msgs::Bool shutdown)
{
    if(shutdown.data)
        ros::shutdown();
}

int Rosm3tTracker::initm3t()
{
  initDone = true;
  //	std::cout << "Debug Mode: " << cvv::debugMode() << std::endl;
  std::string packagePath = ros::package::getPath ("m3t");
  std::string srcPath = packagePath.substr (0, packagePath.size() - 4);
  std::string saveImagesPath = srcPath + "/recordings";
  std::string bodyPath = srcPath + "/resources" + "/models" ;
  const std::filesystem::path directory{bodyPath};

  constexpr bool kUseDepthViewer = false;
  constexpr bool kUseRegionModality = true;
  constexpr bool kUseTextureModality = false;
  constexpr bool kUseDepthModality = false;
  constexpr bool kMeasureOcclusions = false;
  constexpr bool kModelOcclusions = false;
  constexpr bool kVisualizePoseResult = true;
  constexpr bool kSaveImages = false;
  const std::filesystem::path save_directory{""};

  // Set up tracker and renderer geometry
    tracker_ptr = std::make_shared<m3t::Tracker> ("tracker");
    std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr{
    std::make_shared<m3t::RendererGeometry>("renderer geometry")};


  // Set up publisher class
  publisher_ptr = std::make_shared<m3t::RosPublisher> ("pose", *nh);
  tracker_ptr->AddPublisher (publisher_ptr);
  publisher_ptr->AddRosPublisher (posePub);
  // Pose data gets calculated relative to the publisher camera
  publisher_ptr->AddCamera (color_camera_ptr);

/*
  // Set up cameras
  auto color_camera_ptr{
      std::make_shared<m3t::AzureKinectColorCamera>("azure_kinect_color")};
  auto depth_camera_ptr{
      std::make_shared<m3t::AzureKinectDepthCamera>("azure_kinect_depth")};
*/
  
  // Set up viewers
  auto color_viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
      "color_viewer", color_camera_ptr, renderer_geometry_ptr)};
  if (kSaveImages) color_viewer_ptr->StartSavingImages(save_directory, "bmp");
  tracker_ptr->AddViewer(color_viewer_ptr);
  if (kUseDepthViewer) {
    auto depth_viewer_ptr{std::make_shared<m3t::NormalDepthViewer>(
        "depth_viewer", depth_camera_ptr, renderer_geometry_ptr, 0.3f, 1.0f)};
    if (kSaveImages) depth_viewer_ptr->StartSavingImages(save_directory, "bmp");
    tracker_ptr->AddViewer(depth_viewer_ptr);
  }

  // Set up depth renderer
  auto color_depth_renderer_ptr{
      std::make_shared<m3t::FocusedBasicDepthRenderer>(
          "color_depth_renderer", renderer_geometry_ptr, color_camera_ptr)};
   if(useDepthCamera)  
   {     
        auto depth_depth_renderer_ptr{
        std::make_shared<m3t::FocusedBasicDepthRenderer>(
          "depth_depth_renderer", renderer_geometry_ptr, depth_camera_ptr)};
   }
  // Set up silhouette renderer
  auto color_silhouette_renderer_ptr{
      std::make_shared<m3t::FocusedSilhouetteRenderer>(
          "color_silhouette_renderer", renderer_geometry_ptr,
          color_camera_ptr)};

  for (const auto body_name : body_names_) {
    // Set up body
    std::filesystem::path metafile_path{directory / body_name / (body_name + ".yaml")};
    auto body_ptr{std::make_shared<m3t::Body>(body_name, metafile_path)};
    renderer_geometry_ptr->AddBody(body_ptr);
    color_depth_renderer_ptr->AddReferencedBody(body_ptr);
    //depth_depth_renderer_ptr->AddReferencedBody(body_ptr);
    color_silhouette_renderer_ptr->AddReferencedBody(body_ptr);

    // Set up models
    auto region_model_ptr{std::make_shared<m3t::RegionModel>(
        body_name + "_region_model", body_ptr,
        directory / (body_name + "_region_model.bin"))};
    auto depth_model_ptr{std::make_shared<m3t::DepthModel>(
        body_name + "_depth_model", body_ptr,
        directory / (body_name + "_depth_model.bin"))};

    // Set up modalities
    auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
        body_name + "_region_modality", body_ptr, color_camera_ptr,
        region_model_ptr)};
    auto texture_modality_ptr{std::make_shared<m3t::TextureModality>(
        body_name + "_texture_modality", body_ptr, color_camera_ptr,
        color_silhouette_renderer_ptr)};
    if(useDepthCamera)
    {
        auto depth_modality_ptr{std::make_shared<m3t::DepthModality>(
        body_name + "_depth_modality", body_ptr, depth_camera_ptr,
        depth_model_ptr)};
    }
    if (kVisualizePoseResult) {
      region_modality_ptr->set_visualize_pose_result(true);
    }
    if (kMeasureOcclusions) {
      region_modality_ptr->MeasureOcclusions(depth_camera_ptr);
      texture_modality_ptr->MeasureOcclusions(depth_camera_ptr);
     // depth_modality_ptr->MeasureOcclusions();
    }
    if (kModelOcclusions) {
      region_modality_ptr->ModelOcclusions(color_depth_renderer_ptr);
      texture_modality_ptr->ModelOcclusions(color_depth_renderer_ptr);
      //depth_modality_ptr->ModelOcclusions(depth_depth_renderer_ptr);
    }

    // Set up link
    auto link_ptr{std::make_shared<m3t::Link>(body_name + "_link", directory / body_name/(body_name + "_link.yaml"), body_ptr)};
    if (kUseRegionModality) link_ptr->AddModality(region_modality_ptr);
    if (kUseTextureModality) link_ptr->AddModality(texture_modality_ptr);
    //if (kUseDepthModality) link_ptr->AddModality(depth_modality_ptr);

    // Set up optimizer
    auto optimizer_ptr{
        std::make_shared<m3t::Optimizer>(body_name + "_optimizer", link_ptr)};
    tracker_ptr->AddOptimizer(optimizer_ptr);

    // Set up detector
    std::filesystem::path detector_path{directory /body_name/
                                        (body_name + "_manual_detector.yaml")};
    auto detector_ptr{std::make_shared<m3t::ManualDetector>(
        body_name + "_detector", detector_path, optimizer_ptr, color_camera_ptr)};
    tracker_ptr->AddDetector(detector_ptr);
  }
    // Subscribe to GETjag camera
    image_transport::ImageTransport it (*nh);
    imageSubRgb = it.subscribe (cameraTopic + "/image_raw", 10, &Rosm3tTracker::rgbImageCallback, this);
    if (useDepthCamera)
    {
        imageSubDepth = it.subscribe ("/depth_camera/depth/image_raw", 10, &Rosm3tTracker::depthImageCallback, this);
    }
  // Start tracking
  //if (!tracker_ptr->SetUp()) return -1;
  //if (!tracker_ptr->RunTrackerProcess(true, false)) return -1;
  return 0;
}

int Rosm3tTracker::runm3t()
{
    runm3tDone = true;
    if (!tracker_ptr->SetUp())
    {
        return 0;
    }
    m3tThread = std::thread (&m3t::Tracker::RunTrackerProcess, tracker_ptr, false, false, nullptr, nullptr);
    return 0;
}

void Rosm3tTracker::rgbCameraInfoCallback (const sensor_msgs::CameraInfoConstPtr cameraInfo)
{
    // Currently, only initialize the PnP detector once the first camera parameter info arrives
    if (firstInitRgb == false)
    {
        rgbCameraInfoSub.shutdown();
        ROS_INFO ("RGB Camera parameters arrived. Unsubscribing from rgb camera info \n");

        // Set up camera
        color_camera_ptr =
            std::make_shared<m3t::RosCameraColorCamera> ("ros_camera_color", cameraInfo);

        firstInitRgb = true;

        // Initialize the rest if both rgb and depth cameras were initialized
        if (firstInitRgb && (firstInitDepth || !useDepthCamera) && !initDone)
        {
            initm3t();
        }
    }
}

void Rosm3tTracker::depthCameraInfoCallback (const sensor_msgs::CameraInfoConstPtr cameraInfo)
{
    // Currently, only initialize the PnP detector once the first camera parameter info arrives
    if (firstInitDepth == false)
    {
        depthCameraInfoSub.shutdown();
        ROS_INFO ("Depth camera parameters arrived. Unsubscribing from depth camera info \n");

        // Set up camera
        depth_camera_ptr =
            std::make_shared<m3t::RosCameraDepthCamera> ("ros_camera_depth", cameraInfo);

        firstInitDepth = true;

        // Initialize the rest if both rgb and depth cameras were initialized
        if (firstInitRgb && firstInitDepth && !initDone)
        {
            initm3t();
        }
    }
}

void Rosm3tTracker::rgbImageCallback (const sensor_msgs::ImageConstPtr &msg)
{
    if (firstInitRgb == false)
    {
        // Camera parameters have not arrived to the cameraInfoCallback function yet
        return;
    }
    firstImageRgb = true;
    // std::cout << "rgb image arrived" << std::endl;
    // ROS uses callbacks for new images while m3t uses active polling.
    // To make them compatible, the callback function has to fake a buffer for new images.
    cv_ptr_rgb = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);

    // Convert ros image to opencv image
    cv::Mat frame;
    frame = cv_ptr_rgb->image;
    // Add the message header to the publisher object so it can get published
    publisher_ptr->msg_header = msg->header;
    color_camera_ptr->UpdateCapture (frame);

    // Run m3t if the first RGB and depth images arrived (if depth camera is used)
    if (firstImageRgb && (firstImageDepth || !useDepthCamera) && !runm3tDone)
    {
        runm3t();
    }
}

void Rosm3tTracker::depthImageCallback (const sensor_msgs::ImageConstPtr &msg)
{
    if (firstInitDepth == false)
    {
        // Camera parameters have not arrived to the cameraInfoCallback function yet
        return;
    }
    firstImageDepth = true;
    // std::cout << "depth image arrived" << std::endl;
    // ROS uses callbacks for new images while m3t uses active polling.
    // To make them compatible, the callback function has to fake a buffer for new images.
    cv_ptr_depth = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::TYPE_16UC1);

    // Convert ros image to opencv image
    cv::Mat frame;
    frame = cv_ptr_depth->image;
    depth_camera_ptr->UpdateCapture (frame);

    // Run m3t if the first RGB and depth images arrived
    if (firstImageRgb && firstImageDepth && !runm3tDone)
    {
        runm3t();
    }
}
