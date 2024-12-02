/** Written by the Object Handling subgroup
 **/

// #include "MainRos.cpp"
#include <rtb_evaluator.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rtb_evaluator");
  ros::NodeHandle nh;
  ros::CallbackQueue callbackQueue;
  nh.setCallbackQueue(&callbackQueue);

  ROS_INFO("Starting node ...\n");
  std::string packagePath = ros::package::getPath("m3t");
  std::string srcPath = packagePath.substr(0, packagePath.size() - 17);
  auto dataset_directory_ = std::filesystem::path(srcPath)/std::filesystem::path("kinova_gen3")/std::filesystem::path("Gripper");
  auto external_directory_ = std::filesystem::path(srcPath)/std::filesystem::path("kinova_gen3")/std::filesystem::path("Gripper");
  auto object_names = {std::string ("gripper")};
/*  RTBEvaluator rtbevaluator(std::string("evaluator"),
  std::filesystem::path(dataset_directory_)/std::filesystem::path("Gripper")/std::filesystem::path("gripper"), 
  std::filesystem::path(dataset_directory_)/std::filesystem::path("Gripper")/std::filesystem::path("gripper"), 
  object_names,rosru
  {std::string ("")},{std::string (" ")},{1,2,3,4,5});*/
 // Directories
  std::filesystem::path dataset_directory = dataset_directory_;
  std::filesystem::path external_directory = dataset_directory_;
  std::filesystem::path result_path = dataset_directory_;

  // Dataset configuration

  std::vector<std::string> difficulty_levels{"test_easy", "test_medium",
                                             "test_hard"};
  std::vector<std::string> depth_names{"depth_ground_truth",
                                       "depth_azure_kinect",
                                       "depth_active_stereo", "depth_stereo"};
  std::vector<int> sequence_numbers{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  // Run experiments
  RTBEvaluator evaluator{"evaluator",     dataset_directory, external_directory,
                         object_names,    difficulty_levels, depth_names,
                         sequence_numbers};
  evaluator.set_region_modality_setter([&](auto r) {
    r->set_n_lines_max(300);
    r->set_use_adaptive_coverage(true);
    r->set_min_continuous_distance(3.0f);
    r->set_function_length(8);
    r->set_distribution_length(12);
    r->set_function_amplitude(0.43f);
    r->set_function_slope(0.5f);
    r->set_learning_rate(1.3f);
    r->set_scales({9, 7, 5, 2});
    r->set_standard_deviations({25.0f, 15.0f, 10.0f});
    r->set_unconsidered_line_length(0.5f);
    r->set_max_considered_line_length(20.0f);
    if (!r->use_shared_color_histograms()) {
      r->set_n_histogram_bins(16);
      r->set_learning_rate_f(0.2f);
      r->set_learning_rate_b(0.2f);
    }
  });
  evaluator.set_color_histograms_setter([&](auto h) {
    h->set_n_bins(16);
    h->set_learning_rate_f(0.2f);
    h->set_learning_rate_b(0.2f);
  });
  evaluator.set_depth_modality_setter([&](auto d) {
    d->set_n_points_max(300);
    d->set_use_adaptive_coverage(true);
    d->set_use_depth_scaling(true);
    d->set_stride_length(0.008f);
    d->set_considered_distances({0.1f, 0.08f, 0.05f});
    d->set_standard_deviations({0.05f, 0.03f, 0.02f});
  });
  evaluator.set_optimizer_setter([&](auto o) {
    o->set_tikhonov_parameter_rotation(100.0f);
    o->set_tikhonov_parameter_translation(1000.0f);
  });
  evaluator.set_tracker_setter([&](auto t) {
    t->set_n_update_iterations(2);
    t->set_n_corr_iterations(6);
  });
  evaluator.set_evaluation_mode(RTBEvaluator::EvaluationMode::COMBINED);
  evaluator.set_evaluate_external(false);
  evaluator.set_external_results_folder("dart");
  evaluator.set_run_sequentially(true);
  evaluator.set_use_random_seed(false);
  evaluator.set_n_vertices_evaluation(1000);
  evaluator.set_visualize_frame_results(false);
  evaluator.set_visualize_tracking(true);
  evaluator.set_use_shared_color_histograms(true);
  evaluator.set_use_region_checking(true);
  evaluator.set_use_silhouette_checking(true);
  evaluator.SetUp();
  evaluator.Evaluate();
  evaluator.SaveResults(result_path);

  ros::AsyncSpinner spinner(4, &callbackQueue);
  spinner.start();

  //  ros::waitForShutdown();
  ROS_INFO("Done\n");

  std::cout << "GOODBYE ..." << std::endl;
  return 0;
}
