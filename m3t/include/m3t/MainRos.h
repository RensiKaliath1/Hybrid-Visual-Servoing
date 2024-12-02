#ifndef MAIN_ROS_H_
#define MAIN_ROS_H_

// C++
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <array>
#include <fstream>
#include <iostream>
#include <memory>
#include <nanoflann/KDTreeVectorOfVectorAdaptor.hpp>
#include <nanoflann/nanoflann.hpp>
#include <numeric>
#include <random>
#include <string>
#include <vector>
// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
// ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include "ros/package.h"
// Own libraries
#include <filesystem/filesystem.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/generator.h>
#include <m3t/link.h>
#include <m3t/manual_detector.h>
#include <m3t/normal_viewer.h>
#include <m3t/publisher.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/silhouette_renderer.h>
#include <m3t/ros_camera.h>
#include <m3t/ros_publisher.h>
#include <m3t/static_detector.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>

class Rosm3tTracker {
 private:
  static constexpr size_t kNCurveValues = 100;
  static constexpr m3t::Intrinsics kRTBIntrinsics{1000.0f, 1000.0f, 640.0f,
                                                  480.0f,  1280,    960};
  const std::map<std::string, float> kObject2SizeMultiplier{
      {"kinova_gen3", 1.0f},      {"asymmetric_pipestar_without_cap", 1.0f},  {"cube", 1.0f}, {"asymmetric_pipestar_with_cap", 1.0f}};

  typedef KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3f>, float>
      KDTreeVector3f;
 public:
  enum class EvaluationMode {
    INDEPENDENT = 0,
    PROJECTED = 1,
    CONSTRAINED = 2,
    COMBINED = 3
  };

  struct ExecutionTimes {
    float calculate_correspondences = 0.0f;
    float calculate_gradient_and_hessian = 0.0f;
    float calculate_optimization = 0.0f;
    float calculate_results = 0.0f;
    float complete_cycle = 0.0f;
  };

  struct Result {
    int frame_index = 0;
    float add_auc = 0.0f;
    float adds_auc = 0.0f;
    std::array<float, kNCurveValues> add_curve{};
    std::array<float, kNCurveValues> adds_curve{};
    ExecutionTimes execution_times{};
  };

  Rosm3tTracker(ros::NodeHandle *nodehandle);
  // Constructors and setup method
  Rosm3tTracker(const std::string &name,
               const std::filesystem::path &dataset_directory,
               const std::filesystem::path &external_directory,
               const std::vector<std::string> &object_names,
               const std::vector<std::string> &difficulty_levels,
               const std::vector<std::string> &depth_names,
               const std::vector<int> &sequence_numbers);
  bool SetUp();

  // Setters for evaluation
  void set_evaluation_mode(EvaluationMode evaluation_mode);
  void set_evaluate_external(bool evaluate_external);
  void set_external_results_folder(
      const std::filesystem::path &external_results_folder);
  void set_run_sequentially(bool run_sequentially);
  void set_use_random_seed(bool use_random_seed);
  void set_n_vertices_evaluation(int n_vertices_evaluation);
  void set_visualize_tracking(bool visualize_tracking);
  void set_visualize_frame_results(bool visualize_frame_results);
  void StartSavingImages(const std::filesystem::path &save_directory);
  void StopSavingImages();

  // Setters for tracker configuration
  void set_use_shared_color_histograms(bool use_shared_color_histograms);
  void set_use_region_checking(bool use_region_checking);
  void set_use_silhouette_checking(bool use_silhouette_checking);

  // Setters for object setters
  void set_tracker_setter(
      const std::function<void(std::shared_ptr<m3t::Tracker>)> &tracker_setter);
  void set_optimizer_setter(
      const std::function<void(std::shared_ptr<m3t::Optimizer>)>
          &optimizer_setter);
  void set_region_modality_setter(
      const std::function<void(std::shared_ptr<m3t::RegionModality>)>
          &region_modality_setter);
  void set_color_histograms_setter(
      const std::function<void(std::shared_ptr<m3t::ColorHistograms>)>
          &color_histograms_setter);
  void set_region_model_setter(
      const std::function<void(std::shared_ptr<m3t::RegionModel>)>
          &region_model_setter);
  void set_depth_modality_setter(
      const std::function<void(std::shared_ptr<m3t::DepthModality>)>
          &depth_modality_setter);
  void set_depth_model_setter(
      const std::function<void(std::shared_ptr<m3t::DepthModel>)>
          &depth_model_setter);

  // Main methods
  bool Evaluate();
  void SaveResults(std::filesystem::path path) const;

  // Getters
  float add_auc() const;
  float adds_auc() const;
  float execution_time() const;
  std::map<std::string, Result> final_results() const;
  std::mutex mutex_run_tracker{};
 private:

  struct RunConfiguration {
    std::string object_name{};
    std::string difficulty_level{};
    std::string depth_name{};
    std::string sequence_name{};
    std::string title{};
  };

  struct SequenceResult {
    std::string object_name{};
    std::string difficulty_level{};
    std::string depth_name{};
    std::string sequence_name{};
    std::string title{};
    std::vector<Result> frame_results{};
    Result average_result{};
  };

  // Set up methods
  void setParameters(ros::NodeHandle& nh);
  void CreateRunConfigurations();
  bool LoadObjectBodies(
      const std::string &object_name,
      std::vector<std::shared_ptr<m3t::Body>> *body_ptrs) const;
  bool GenerateModels(std::vector<std::shared_ptr<m3t::Body>> &body_ptrs,
                      const std::string &object_name);
  void GenderateReducedVertices(
      std::vector<std::shared_ptr<m3t::Body>> &body_ptrs);
  void GenerateKDTrees(std::vector<std::shared_ptr<m3t::Body>> &body_ptrs);
  bool LoadEvaluationData(const std::string &object_name);

  // Helper methods to run evaluation
  bool EvaluateRunConfiguration(
      const RunConfiguration &run_configuration,
      const std::shared_ptr<m3t::Tracker> &tracker_ptr,
      SequenceResult *sequence_results) const;

  bool RunTracker(
    const std::shared_ptr<m3t::Tracker> &tracker_ptr);
bool RunTrackerObj(
    const std::shared_ptr<m3t::Tracker> &tracker_ptr);   
  bool SetUpTracker(
      const std::string &object_name,
      const std::shared_ptr<m3t::RendererGeometry> &renderer_geometry_ptr,
      std::shared_ptr<m3t::Tracker> tracker_ptr, std::shared_ptr<m3t::ColorCamera> camera_ptr, std::shared_ptr<m3t::DepthCamera> depth_camera_ptr);
  bool SetRunConfiguration(
      const RunConfiguration &run_configuration,
      const std::shared_ptr<m3t::Tracker> &tracker_ptr) const;
  bool LoadPoses(
      const std::filesystem::path &path, int start_index,
      std::vector<std::vector<m3t::Transform3fA>> *body2world_poses_sequence,
      std::vector<float> *execution_times = nullptr) const;
  void SetBodyAndJointPoses(
      const std::vector<m3t::Transform3fA> &body2world_poses,
      const std::shared_ptr<m3t::Tracker> &tracker_ptr) const;
  void SetBodyAndJointPoses(
      const std::vector<m3t::Transform3fA> &body2world_poses,
      const std::shared_ptr<m3t::Link> &link_ptr,
      const std::shared_ptr<m3t::Link> &parent_link_ptr) const;
  void SetBodyAndJointPosesConstrained(
      const std::vector<m3t::Transform3fA> &body2world_poses,
      const std::shared_ptr<m3t::Link> &root_link_ptr) const;
  void ExecuteMeasuredTrackingCycle(
      const std::shared_ptr<m3t::Tracker> &tracker_ptr, int iteration,
      ExecutionTimes *execution_times) const;
  void ExecuteViewingCycle(const std::shared_ptr<m3t::Tracker> &tracker_ptr,
                           int iteration) const;
  void CalculatePoseResults(
      const std::shared_ptr<m3t::Tracker> &tracker_ptr,
      const std::vector<m3t::Transform3fA> &gt_body2world_poses,
      Result *result) const;

  // Helper methods for the calculation of results
  Result CalculateAverageResult(
      const std::vector<std::string> &object_names,
      const std::vector<std::string> &difficulty_levels,
      const std::vector<std::string> &depth_names) const;
  static Result CalculateAverageResult(const std::vector<Result> &results);
  static Result SumResults(const std::vector<Result> &results);
  static Result DivideResult(const Result &result, size_t n);
  static void VisualizeResult(const Result &result, const std::string &title);

  // General helper methods
  static float ElapsedTime(
      const std::chrono::high_resolution_clock::time_point &begin_time);

  // Internal data objects
  std::vector<RunConfiguration> run_configurations_{};
  std::array<float, kNCurveValues> thresholds_{};
  std::map<std::string, int> body_name2idx_map_{};
  std::vector<std::shared_ptr<m3t::RegionModel>> region_model_ptrs_{};
  std::vector<std::shared_ptr<m3t::DepthModel>> depth_model_ptrs_{};
  std::vector<std::vector<Eigen::Vector3f>> reduced_vertices_vector_{};
  std::vector<std::unique_ptr<KDTreeVector3f>> kdtree_ptr_vector_{};
  std::vector<std::vector<int>> ids_combined_bodies_{};
  float error_threshold_;
  float max_model_contour_length_;
  float max_model_surface_area_;
  std::vector<SequenceResult> results_{};
  std::map<std::string, Result> final_results_{};

  // Parameters for RTB dataset
  std::filesystem::path dataset_directory_{};
  std::filesystem::path external_directory_{};
  std::vector<std::string> object_names_{};
  const std::vector<std::string> difficulty_levels_{};
  const std::vector<std::string> depth_names_{};
  std::vector<int> sequence_numbers_{};

  // Parameters for tracker configuration
  bool use_shared_color_histograms_ = true;
  bool use_region_checking_ = true;
  bool use_silhouette_checking_ = false;

  // Setters for object setters
  std::function<void(std::shared_ptr<m3t::Tracker>)> tracker_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::Optimizer>)> optimizer_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::RegionModality>)>
      region_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<m3t::ColorHistograms>)>
      color_histograms_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<m3t::RegionModel>)> region_model_setter_{
      [](auto) {}};
  std::function<void(std::shared_ptr<m3t::DepthModality>)>
      depth_modality_setter_{[](auto) {}};
  std::function<void(std::shared_ptr<m3t::DepthModel>)> depth_model_setter_{
      [](auto) {}};

  // Parameters for evaluation
  std::string name_{};
  EvaluationMode evaluation_mode_ = EvaluationMode::COMBINED;
  bool evaluate_external_ = false;
  std::filesystem::path external_results_folder_;
  bool run_sequentially_ = true;
  bool use_random_seed_ = false;
  int n_vertices_evaluation_ = -1;
  bool visualize_tracking_ = true;
  bool visualize_frame_results_ = false;
  bool save_images_ = false;
  std::filesystem::path save_directory_{};
  bool set_up_ = false;

  int initm3t();
  std::shared_ptr<m3t::Tracker> runm3t(std::string object_name, std::shared_ptr<m3t::RosCameraColorCamera> color_camera, std::shared_ptr<m3t::RosCameraDepthCamera> depth_camera);

  // Callbacks
  void rgbImageCallback(const sensor_msgs::ImageConstPtr &msg);
  void depthImageCallback(const sensor_msgs::ImageConstPtr &msg);
  void rgbCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr cameraInfo);
  void depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr cameraInfo);

  // Subscribers
  image_transport::Subscriber imageSubRgb;
  image_transport::Subscriber imageSubDepth;
  ros::Subscriber rgbCameraInfoSub;
  ros::Subscriber depthCameraInfoSub;

  std::string colorCameraImageTopic;
  std::string depthCameraImageTopic;
  std::string colorCameraInfoTopic;
  std::string depthCameraInfoTopic;
  std::string srcPath;
  // Publishers
  image_transport::Publisher pose_estimator_pub;
  std::shared_ptr<m3t::RosPublisher> publisher_ptr;
  ros::Publisher posePub;
  ros::Publisher poseArmPub;
  // Internal state variables
  bool firstInitRgb = false;
  bool firstInitDepth = false;
  bool firstImageRgb = false;
  bool firstImageDepth = false;
  bool initDone = false;
  bool runm3tDone = false;
  bool useDepthCamera = true;
  ros::NodeHandle *nh;
  std::vector<std::string> object_names;
  cv_bridge::CvImagePtr cv_ptr_rgb;
  cv_bridge::CvImagePtr cv_ptr_depth;

  std::shared_ptr<m3t::RosCameraColorCamera> color_camera_ptr;
  std::shared_ptr<m3t::RosCameraColorCamera> color_camera_ptr_obj;
  std::shared_ptr<m3t::RosCameraDepthCamera> depth_camera_ptr;
  std::shared_ptr<m3t::RosCameraDepthCamera> depth_camera_ptr_obj;
  std::thread m3tThread;
  std::thread m3tThread_obj;
  std::mutex mutex_callback{};
};

#endif
