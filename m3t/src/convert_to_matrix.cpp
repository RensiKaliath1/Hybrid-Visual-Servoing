#include <cv_bridge/cv_bridge.h>
#include <filesystem/filesystem.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelState.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include "ros/node_handle.h"
#include "ros/package.h"
#include "tf/LinearMath/Vector3.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

// Works with both gazebo simulator and hardware. Just change the image topic in dexterity_params.yaml
// change dataset path if required
const std::string camera = std::string("camera");

bool use_tf = true;
class ConvertToMatrix {
 public:
  ros::NodeHandle n;

  gazebo_msgs::GetLinkState base_link_state;
  gazebo_msgs::GetLinkState shoulder_link_state;
  gazebo_msgs::GetLinkState bicep_link_state;
  gazebo_msgs::GetLinkState forearm_link_state;
  gazebo_msgs::GetLinkState bracelet_link_state;
  gazebo_msgs::GetLinkState spherical_wrist_1_link_state;
  gazebo_msgs::GetLinkState spherical_wrist_2_link_state;

  gazebo_msgs::GetLinkState shoulder_base_state;
  gazebo_msgs::GetLinkState bicep_shoulder_state;
  gazebo_msgs::GetLinkState forearm_bicep_state;
  gazebo_msgs::GetLinkState spherical_wrist_1_forearm_state;
  gazebo_msgs::GetLinkState spherical_wrist_2_spherical_wrist_1_state;
  gazebo_msgs::GetLinkState bracelet_spherical_wrist_2_state;
 
  gazebo_msgs::GetModelState camera_to_base_link_state;
  gazebo_msgs::GetModelState camera_to_world_state;

  ros::ServiceClient client_base_link;
  ros::ServiceClient client_shoulder_link;
  ros::ServiceClient client_bicep_link;
  ros::ServiceClient client_forearm_link;
  ros::ServiceClient client_spherical_wrist_1_link;
  ros::ServiceClient client_spherical_wrist_2_link;
  ros::ServiceClient client_bracelet_link;

  ros::ServiceClient client_shoulder_base;
  ros::ServiceClient client_bicep_shoulder;
  ros::ServiceClient client_forearm_bicep;
  ros::ServiceClient client_spherical_wrist_1_forearm;
  ros::ServiceClient client_spherical_wrist_2_spherical_wrist_1;
  ros::ServiceClient client_bracelet_spherical_wrist_2;

  ros::ServiceClient client_camera_to_base_link;
  ros::ServiceClient client_camera_to_world;

  std::string tf_prefix ="";
  std::string cameraSensorFrame = "camera_rgb_optical_frame";
  std::string sourcePath;
  image_transport::Subscriber imageSub;
  bool first = true;
  bool firstYaml = true;
  bool firstConstraintYaml = true;

  ConvertToMatrix();
  void imageCallback(const sensor_msgs::ImageConstPtr& camImage);
  void configWriter();
  		//TF Buffer and Listener
  tf::TransformListener listener;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tfListener {tf_buffer};
  void setParameters(ros::NodeHandle& n);
  geometry_msgs::PoseStamped calculateTransformStampPosition (const geometry_msgs::PoseStamped &geoPoseStampedIn, const std::string targetFrame);
};

void ConvertToMatrix::setParameters(ros::NodeHandle& n)
{
    std::string tf_prefix_param;
    std::string camera_sensor_frame_id_param;
    std::string source_path;
    n.getParam("/Dexterity/hardware_parameters/configPath", source_path);
    sourcePath = source_path;
    n.getParam("/Dexterity/hardware_parameters/tf_prefix", tf_prefix_param);
    tf_prefix = tf_prefix_param;
    n.getParam("/Dexterity/hardware_parameters/camera_sensor_frame_id", camera_sensor_frame_id_param);
    cameraSensorFrame = camera_sensor_frame_id_param;
    std::cout<<"the sensor frame is "<<cameraSensorFrame;
}
ConvertToMatrix::ConvertToMatrix() {

  listener.waitForTransform(tf_prefix+"base_link", cameraSensorFrame, ros::Time(0), ros::Duration(1));
  client_base_link =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_shoulder_link =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_bicep_link =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_forearm_link =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_spherical_wrist_1_link =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_spherical_wrist_2_link =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_bracelet_link =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

  client_camera_to_base_link =
      n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  client_camera_to_world =
      n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  client_shoulder_base =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_bicep_shoulder =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_forearm_bicep =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_spherical_wrist_1_forearm=
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_spherical_wrist_2_spherical_wrist_1 =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  client_bracelet_spherical_wrist_2 =
      n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");


  base_link_state.request.link_name = "base_link";
  base_link_state.request.reference_frame = camera;

  shoulder_link_state.request.link_name = "shoulder_link";
  shoulder_link_state.request.reference_frame = camera;

  bicep_link_state.request.link_name = "bicep_link";
  bicep_link_state.request.reference_frame = camera;

  forearm_link_state.request.link_name = "forearm_link";
  forearm_link_state.request.reference_frame = camera;

  spherical_wrist_1_link_state.request.link_name = "spherical_wrist_1_link";
  spherical_wrist_1_link_state.request.reference_frame = camera;

  spherical_wrist_2_link_state.request.link_name = "spherical_wrist_2_link";
  spherical_wrist_2_link_state.request.reference_frame = camera;

  bracelet_link_state.request.link_name = "bracelet_link";
  bracelet_link_state.request.reference_frame = camera;


  shoulder_base_state.request.link_name = "shoulder_link";
  shoulder_base_state.request.reference_frame = "base_link";

  bicep_shoulder_state.request.link_name = "bicep_link";
  bicep_shoulder_state.request.reference_frame = "shoulder_link";

  forearm_bicep_state.request.link_name = "forearm_link";
  forearm_bicep_state.request.reference_frame = "bicep_link";

  spherical_wrist_1_forearm_state.request.link_name = "spherical_wrist_1_link";
  spherical_wrist_1_forearm_state.request.reference_frame = "forearm_link";

  spherical_wrist_2_spherical_wrist_1_state.request.link_name = "spherical_wrist_2_link";
  spherical_wrist_2_spherical_wrist_1_state.request.reference_frame = "spherical_wrist_1_link";

  bracelet_spherical_wrist_2_state.request.link_name = "bracelet_link";
  bracelet_spherical_wrist_2_state.request.reference_frame = "spherical_wrist_2_link";

  camera_to_base_link_state.request.model_name = "my_gen3";
  camera_to_base_link_state.request.relative_entity_name = camera;

  camera_to_world_state.request.model_name = camera;
  camera_to_world_state.request.relative_entity_name = "world";

  setParameters(n);
  image_transport::ImageTransport imageTransport(n);
  imageSub = imageTransport.subscribe(cameraSensorFrame + std::string("/camera1/camera1/color/image_raw"), 50,
                                      &ConvertToMatrix::imageCallback, this);
}

 void ConvertToMatrix::imageCallback(const sensor_msgs::ImageConstPtr& camImage)
 {

 }
geometry_msgs::PoseStamped ConvertToMatrix::calculateTransformStampPosition (const geometry_msgs::PoseStamped &geoPoseStampedIn, const std::string targetFrame)
{
    geometry_msgs::PoseStamped geoPoseStampedOut;
    try
    {
       // listener.waitForTransform("/camera_base_link", targetFrame, ros::Time(0), ros::Duration(1));
        listener.transformPose (targetFrame, geoPoseStampedIn, geoPoseStampedOut);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN ("%s", ex.what());
        ros::Duration (1.0).sleep();
    }
    return geoPoseStampedOut;
}

void ConvertToMatrix::configWriter()
   {


  auto writeToYamlStaticDetectorAndCameraToWord = [&srcPath = this->sourcePath](tf::Matrix3x3 rotation,
                                tf2::Vector3 translation, bool cameraToWorld) 
  {
    std::string packagePath = ros::package::getPath("m3t");
    //std::string srcPath = packagePath.substr(0, packagePath.size() - 8);
    std::filesystem::path dataset = std::filesystem::path(srcPath) /
                              std::filesystem::path("kinova_gen3") /
                              "tracker_config" / "static_detector.yaml";
    std::filesystem::path cameraToWorldPath = std::filesystem::path(srcPath) /
                              std::filesystem::path("kinova_gen3") /
                              "tracker_config" / "camera_config.yaml";
    std::filesystem::path path =dataset;
    if(cameraToWorld)
        path = cameraToWorldPath;
    else
        path = dataset;

    std::ofstream fileSystem;
    fileSystem.open(path);
 
    fileSystem << "%YAML:1.2";
    if(!cameraToWorld)
        fileSystem << "\nlink2world_pose: !!opencv-matrix";
    else
      fileSystem << "\n camera2world_pose: !!opencv-matrix";
    fileSystem << "\n  rows: 4";
    fileSystem << "\n  cols: 4"; 
    fileSystem << "\n  dt: d ";
    fileSystem << "\n  data:[";
    fileSystem << rotation[0][0]<< "," << rotation[0][1] << "," << rotation[0][2]
       << "," << translation[0] << ","
       << rotation[1][0] << "," << rotation[1][1] << "," << rotation[1][2]
       << "," << translation[1] << ","
       << rotation[2][0] << "," << rotation[2][1] << "," << rotation[2][2]
       << "," << translation[2] << ","
       << "0,0,0,1]";
    fileSystem.close();

  };

  auto writeToConstraint = [&firstConstraintYaml = this->firstConstraintYaml, &srcPath = this->sourcePath](tf::Matrix3x3 rotation, tf::Matrix3x3 rotationBodyToJoint2, tf::Vector3 translation, tf::Vector3 translation2, std::string fileName, int body_id) 
  {
    if(!firstConstraintYaml)
        return;
    auto dataset = std::filesystem::path(srcPath) /
                              std::filesystem::path("kinova_gen3") /
                              "tracker_config" / fileName;
    std::filesystem::path path = dataset;
    std::ofstream fileSystem;
    fileSystem.open(path);
    std::cout<<"\n \n writing to yaml constraints body id "<< body_id;
    fileSystem << "%YAML:1.2";


    fileSystem << "\nbody12joint1_pose: !!opencv-matrix";
    fileSystem << "\n rows: 4";
    fileSystem << "\n cols: 4"; 
    fileSystem << "\n dt: d ";
    fileSystem << "\n data:[";
    fileSystem << rotation[0][0]<< "," << rotation[0][1] << "," << rotation[0][2]
       << "," << translation[0] << ","
       << rotation[1][0] << "," << rotation[1][1] << "," << rotation[1][2]
       << "," << translation[1] << ","
       << rotation[2][0] << "," << rotation[2][1] << "," << rotation[2][2]
       << "," << translation[2] << ","
       << "0,0,0,1]";
    /*fileSystem << 1<< "," << 0 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 1 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 0 << "," << 1
       << "," << 0 << ","
       << "0,0,0,1]";*/
    fileSystem << "\nbody22joint2_pose: !!opencv-matrix";
    fileSystem << "\n rows: 4";
    fileSystem << "\n cols: 4"; 
    fileSystem << "\n dt: d ";
    fileSystem << "\n data:[";
    fileSystem << 1<< "," << 0 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 1 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 0 << "," << 1
       << "," << 0 << ","
       << "0,0,0,1]";
   /* fileSystem << 1<< "," << 0 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 1 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 0 << "," << 1
       << "," << 0 << ","
       << "0,0,0,1]";*/
    if (body_id == 2|| body_id == 5 || body_id == 7 )
        fileSystem << "\nconstraint_directions: [1, 1, 0, 1, 1, 1]";
    else if(body_id == 3 || body_id == 4 || body_id == 6)
        fileSystem << "\nconstraint_directions: [0, 0, 1, 1, 1, 1]";
    if(body_id == 7)
        firstConstraintYaml = false;
    fileSystem.close();
  };

  auto writeToYamlJoint = [&firstYaml = this->firstYaml, &srcPath = this->sourcePath](tf::Matrix3x3 rotation, tf::Matrix3x3 rotationBodyToJoint, tf::Vector3 translation, std::string fileName, int body_id) 
  {
    if(!firstYaml)
        return;
    std::string packagePath = ros::package::getPath("m3t");
    auto dataset = std::filesystem::path(srcPath) /
                              std::filesystem::path("kinova_gen3") /
                              "tracker_config" / fileName;
    std::filesystem::path path = dataset;
    std::ofstream fileSystem;
    fileSystem.open(path);
    std::cout<<"\n \n writing to yaml joint body id "<< body_id;
    fileSystem << "%YAML:1.2";
    if (body_id == 1 || body_id == 4 || body_id == 6 || body_id == 7)
        fileSystem << "\nfree_directions: [0, 0, 1, 0, 0, 0]";
    else if(body_id == 2 || body_id == 3 || body_id == 5)
        fileSystem << "\nfree_directions: [1, 1, 0, 0, 0, 0]";

    fileSystem << "\nbody2joint_pose: !!opencv-matrix";
    fileSystem << "\n rows: 4";
    fileSystem << "\n cols: 4"; 
    fileSystem << "\n dt: d ";
    fileSystem << "\n data:[";
    /*fileSystem << rotationBodyToJoint[0][0]<< "," << rotationBodyToJoint[0][1] << "," << rotationBodyToJoint[0][2]
       << "," << 0 << ","
       << rotationBodyToJoint[1][0] << "," << rotationBodyToJoint[1][1] << "," << rotationBodyToJoint[1][2]
       << "," << 0 << ","
       << rotationBodyToJoint[2][0] << "," << rotationBodyToJoint[2][1] << "," << rotationBodyToJoint[2][2]
       << "," << 0 << ","
       << "0,0,0,1]";*/
    fileSystem << 1<< "," << 0 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 1 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 0 << "," << 1
       << "," << 0 << ","
       << "0,0,0,1]";
    fileSystem << "\njoint2parent_pose: !!opencv-matrix";
    fileSystem << "\n rows: 4";
    fileSystem << "\n cols: 4"; 
    fileSystem << "\n dt: d ";
    fileSystem << "\n data:[";
    fileSystem << rotation[0][0]<< "," << rotation[0][1] << "," << rotation[0][2]
       << "," << translation[0] << ","
       << rotation[1][0] << "," << rotation[1][1] << "," << rotation[1][2]
       << "," << translation[1] << ","
       << rotation[2][0] << "," << rotation[2][1] << "," << rotation[2][2]
       << "," << translation[2] << ","
       << "0,0,0,1]";
   /* fileSystem << 1<< "," << 0 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 1 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 0 << "," << 1
       << "," << 0 << ","
       << "0,0,0,1]";*/
    if(body_id == 7)
        firstYaml = false;
    fileSystem.close();
  };

    std::string packagePath = ros::package::getPath("m3t");
    std::string srcPath = sourcePath;
    std::filesystem::path path  = std::filesystem::path(srcPath) /
                            std::filesystem::path("kinova_gen3") /
                            "scene_gt.json";
    std::fstream fs; 
    fs.open(path);
    fs << "{\"0\":[";

auto writeToJson = [&first = this->first, &fs]
                            (tf::Matrix3x3 rotation,
                            tf::Vector3 translation, int body_id,
                            int index) 
{

    if(!first)
        return;
    std::cout<<"\n \nbody id "<< body_id;

    fs << "{\"cam_R_m2c\":[";
 /* fs<< 1<< "," << 0 << "," << 0
       << "," << 0 << ","
       << 1 << "," << 0
       << "," << 0 << ","
       << 0 << "," << 1;*/
       
    for (int i = 0; i < 3; ++i) {
        for(int j = 0; j<3; j++)
            {
                fs << rotation[i][j];
                if (i < 3 && j<3) fs << ",";
            }
    }
    fs << "],\"cam_t_m2c\":[";
    for (int i = 0; i < 3; ++i) {
      fs << translation[i];
      if (i < 2) fs << ",";
    }
    fs << "],\"obj_id\": ";
 
    fs << body_id << "}";
     if(body_id != 7)
        fs<<",";
    else
    {
        first = false;
        fs <<"]}";
        fs.close();
        //ros::shutdown();
    }
  };

auto ArmToCameraSensorLinkTransformGazebo = [&] (ros::ServiceClient client, gazebo_msgs::GetLinkState state, std::string armLink, int body_id, int l)
 {
    client.call(state);
    geometry_msgs::Quaternion arm_link_to_camera_state_orientation = state.response.link_state.pose.orientation;
    geometry_msgs::Point arm_link_to_camera_state_position = state.response.link_state.pose.position;

    geometry_msgs::PoseStamped PoseRelativeToCameraLink;
    PoseRelativeToCameraLink.header.stamp = ros::Time(0);                  
    PoseRelativeToCameraLink.header.frame_id = ""; 
    PoseRelativeToCameraLink.pose.position  = arm_link_to_camera_state_position;
    PoseRelativeToCameraLink.pose.orientation = arm_link_to_camera_state_orientation;
    
    auto transformedPose = calculateTransformStampPosition (PoseRelativeToCameraLink, cameraSensorFrame);
    tf2::Vector3 positionRelativeToCameraSensorLink;
    tf2::Quaternion quaternionRelativeToCameraLink;
	tf2::fromMsg(transformedPose.pose.orientation, quaternionRelativeToCameraLink);
    tf2::fromMsg(transformedPose.pose.position, positionRelativeToCameraSensorLink);

    double roll, pitch, yaw;
    tf2::Matrix3x3 rotationRelativeToCameraSensorLink;
    rotationRelativeToCameraSensorLink.setRotation(quaternionRelativeToCameraLink);
    rotationRelativeToCameraSensorLink.getRPY(roll, pitch, yaw);
    tf::Matrix3x3 rotationCameraSensorLink;
    rotationCameraSensorLink.setRPY(roll, pitch, yaw);



    if(body_id == 0)
        writeToYamlStaticDetectorAndCameraToWord(rotationCameraSensorLink, positionRelativeToCameraSensorLink, false);
    if(!((std::abs(positionRelativeToCameraSensorLink[0]-0) < 1e-3) && (std::abs(positionRelativeToCameraSensorLink[1]-0) < 1e-3) && (std::abs(positionRelativeToCameraSensorLink[2]-0) < 1e-3)))
        writeToJson(rotationCameraSensorLink, tf::Vector3{positionRelativeToCameraSensorLink[0], positionRelativeToCameraSensorLink[1], positionRelativeToCameraSensorLink[2]}, body_id, 0);
  };


auto ArmLinkToArmLinkGazebo = [&](ros::ServiceClient client, gazebo_msgs::GetLinkState state, std::string ParentLink, std::string ChildLink, int body_id)
{
    client.call(state);
    geometry_msgs::Quaternion child_to_parent_orientation = state.response.link_state.pose.orientation;
    geometry_msgs::Point child_to_parent_position = state.response.link_state.pose.position;


    tf2::Vector3 positionRelativeToCameraSensorLink;
    tf2::Quaternion quaternionRelativeToCameraLink;
	tf2::fromMsg(child_to_parent_orientation, quaternionRelativeToCameraLink);
    tf2::fromMsg(child_to_parent_position, positionRelativeToCameraSensorLink);

    double roll, pitch, yaw;
    tf2::Matrix3x3 rotationRelativeToCameraSensorLink;
    rotationRelativeToCameraSensorLink.setRotation(quaternionRelativeToCameraLink);
    rotationRelativeToCameraSensorLink.getRPY(roll, pitch, yaw);
    tf::Matrix3x3 rotationCameraSensorLink;
    rotationCameraSensorLink.setRPY(roll, pitch, yaw);

    //if(!((std::abs(positionRelativeToCameraSensorLink[0]-0) < 1e-3) && (std::abs(positionRelativeToCameraSensorLink[1]-0) < 1e-3) && (std::abs(positionRelativeToCameraSensorLink[2]-0) < 1e-3)))
  //  writeToYamlJoint(rotationCameraSensorLink, tf::Vector3{positionRelativeToCameraSensorLink[0], positionRelativeToCameraSensorLink[1], positionRelativeToCameraSensorLink[2]}, std::string(ChildLink+".yaml"), body_id);
};


auto ArmLinkToArmLinkTF = [&](std::string ParentLink, std::string ChildLink, int body_id)   
{
    tf::StampedTransform transformParent;
    tf::StampedTransform transformCameraSensor;

    try 
    {
        listener.lookupTransform(ChildLink, ParentLink, ros::Time(0), transformParent);
        auto transform = transformParent;
        auto rotation = transform.getBasis() ;
        auto translation = transform.getOrigin();

        tf::Matrix3x3 transformMatrix;
        transformMatrix.setRPY(-1.57, 0,0);
        tf::Transform transformation(transformMatrix, tf::Vector3(0,0,0));

        geometry_msgs::PoseStamped transformedPoseEachLink;
        transformedPoseEachLink.header.frame_id = tf_prefix + "base_link";
        transformedPoseEachLink.header.stamp = ros::Time::now();

        transformedPoseEachLink.pose.position.x = transformation.getOrigin().x();
        transformedPoseEachLink.pose.position.y =  transformation.getOrigin().y();
        transformedPoseEachLink.pose.position.z =  transformation.getOrigin().z();


        transformedPoseEachLink.pose.orientation.x = transformation.getRotation().x();
        transformedPoseEachLink.pose.orientation.y = transformation.getRotation().y();
        transformedPoseEachLink.pose.orientation.z = transformation.getRotation().z();
        transformedPoseEachLink.pose.orientation.w = transformation.getRotation().w();

    
        geometry_msgs::PoseStamped transformedPose;
        auto required_transform = tf_buffer.lookupTransform(ChildLink, transformedPoseEachLink.header.frame_id, ros::Time(0));
        //auto transformedPose = calculateTransformStampPosition(transformedPoseEachLink, ChildLink);
        tf2::doTransform(transformedPoseEachLink.pose,transformedPose.pose,required_transform);
        
        tf2::Vector3 positionRelativeToCameraSensorLink;
        tf2::Quaternion quaternionRelativeToCameraLink;
	    tf2::fromMsg(transformedPose.pose.orientation, quaternionRelativeToCameraLink);
        tf2::fromMsg(transformedPose.pose.position, positionRelativeToCameraSensorLink);

        double roll, pitch, yaw;
        tf2::Matrix3x3 rotationRelativeToCameraSensorLink;
        rotationRelativeToCameraSensorLink.setRotation(quaternionRelativeToCameraLink);
        rotationRelativeToCameraSensorLink.getRPY(roll, pitch, yaw);
        tf::Matrix3x3 rotationCameraSensorLink;
        rotationCameraSensorLink.setRPY(roll, pitch, yaw);

       // auto translation = transformCameraSensorLink.getOrigin();
        auto rotationBodyToJoint =   rotationCameraSensorLink;

    if(!((std::abs(translation[0]-0) < 1e-3) && (std::abs(translation[1]-0) < 1e-3) && (std::abs(translation[2]-0) < 1e-3)))
    {
        writeToYamlJoint(rotation, rotationBodyToJoint, translation,  std::string(ChildLink+".yaml").erase(0,tf_prefix.size()), body_id);
        if(body_id == 2)
            writeToConstraint(rotation, rotationBodyToJoint, translation, translation, std::string("constraint_1_2.yaml"), body_id);
        else if(body_id == 3)
            writeToConstraint(rotation, rotationBodyToJoint, translation, translation, std::string("constraint_2_3.yaml"), body_id);
        else if(body_id == 4)
            writeToConstraint(rotation, rotationBodyToJoint, translation, translation, std::string("constraint_3_4.yaml"), body_id);
        else if(body_id == 5)
            writeToConstraint(rotation, rotationBodyToJoint, translation, translation, std::string("constraint_4_5.yaml"), body_id);
        else if(body_id == 6)
            writeToConstraint(rotation, rotationBodyToJoint, translation, translation, std::string("constraint_5_6.yaml"), body_id);
        else if(body_id == 7)
            writeToConstraint(rotation, rotationBodyToJoint, translation, translation, std::string("constraint_6_7.yaml"), body_id);
    }
    } 
    catch (tf::TransformException &ex)
    {
        ROS_WARN ("%s", ex.what());
        ros::Duration (1.0).sleep();
    }

};

auto ArmLinkToCameraSensorLinkTF = [&](std::string ChildLink, int body_id)
{
    tf::StampedTransform transformCameraSensorLink;
    tf::StampedTransform transformCameraLink;
    tf::StampedTransform transformBaseChildLink;
    try 
    {

     listener.lookupTransform(cameraSensorFrame, ChildLink, ros::Time(0), transformCameraSensorLink);
     tf::Matrix3x3 transformMatrix;
     transformMatrix.setRPY(-1.57, 0,0);
  
     tf::Transform transformation(transformMatrix, tf::Vector3(0,0,0));

    geometry_msgs::PoseStamped transformedPoseEachLink;
    transformedPoseEachLink.header.frame_id = tf_prefix + "base_link";
    transformedPoseEachLink.header.stamp = ros::Time::now();

    transformedPoseEachLink.pose.position.x = transformation.getOrigin().x();
    transformedPoseEachLink.pose.position.y =  transformation.getOrigin().y();
    transformedPoseEachLink.pose.position.z =  transformation.getOrigin().z();


    transformedPoseEachLink.pose.orientation.x = transformation.getRotation().x();
    transformedPoseEachLink.pose.orientation.y = transformation.getRotation().y();
    transformedPoseEachLink.pose.orientation.z = transformation.getRotation().z();
    transformedPoseEachLink.pose.orientation.w = transformation.getRotation().w();

  
    geometry_msgs::PoseStamped transformedPose;
    auto required_transform = tf_buffer.lookupTransform(cameraSensorFrame, transformedPoseEachLink.header.frame_id, ros::Time(0));
    //auto transformedPose = calculateTransformStampPosition(transformedPoseEachLink, ChildLink);
    tf2::doTransform(transformedPoseEachLink.pose,transformedPose.pose,required_transform);

    tf2::Vector3 positionRelativeToCameraSensorLink;
    tf2::Quaternion quaternionRelativeToCameraLink;
	tf2::fromMsg(transformedPose.pose.orientation, quaternionRelativeToCameraLink);
    tf2::fromMsg(transformedPose.pose.position, positionRelativeToCameraSensorLink);

    double roll, pitch, yaw;
    tf2::Matrix3x3 rotationRelativeToCameraSensorLink;
    rotationRelativeToCameraSensorLink.setRotation(quaternionRelativeToCameraLink);
    rotationRelativeToCameraSensorLink.getRPY(roll, pitch, yaw);
    tf::Matrix3x3 rotationCameraSensorLink;
    rotationCameraSensorLink.setRPY(roll, pitch, yaw);

    auto translation = transformCameraSensorLink.getOrigin();
    auto rotation =   rotationCameraSensorLink;

    if(body_id == 0)
        writeToYamlStaticDetectorAndCameraToWord(rotation, tf2::Vector3{translation[0], translation[1], translation[2]}, false);
     if(!((std::abs(translation[0]-0) < 1e-3) && (std::abs(translation[1]-0) < 1e-3) && (std::abs(translation[2]-0) < 1e-3)))
     writeToJson(rotation, translation, body_id, 0);
    } 
    catch (tf::TransformException &ex)
    {
        ROS_WARN ("%s", ex.what());
        ros::Duration (1.0).sleep();
    }
   

};

auto writeToConstraints = [&](std::string ParentLink, std::string ChildLink, std::string filename, int body_id)
{
        tf::StampedTransform transformParent;
      //  listener.lookupTransform("ChildLink", ParentLink, ros::Time(0), transformParent);
        auto transform = transformParent;
        auto rotation = transform.getBasis() ;
        auto translationBodyToJoint1 = transform.getOrigin();

        tf::Matrix3x3 transformMatrix;
        transformMatrix.setRPY(-1.57, 0,0);
        tf::Transform transformation(transformMatrix, tf::Vector3(0,0,0));

        geometry_msgs::PoseStamped transformedPoseEachLink;
        transformedPoseEachLink.header.frame_id = tf_prefix + "base_link";
        transformedPoseEachLink.header.stamp = ros::Time::now();

        transformedPoseEachLink.pose.position.x = transformation.getOrigin().x();
        transformedPoseEachLink.pose.position.y =  transformation.getOrigin().y();
        transformedPoseEachLink.pose.position.z =  transformation.getOrigin().z();


        transformedPoseEachLink.pose.orientation.x = transformation.getRotation().x();
        transformedPoseEachLink.pose.orientation.y = transformation.getRotation().y();
        transformedPoseEachLink.pose.orientation.z = transformation.getRotation().z();
        transformedPoseEachLink.pose.orientation.w = transformation.getRotation().w();

        geometry_msgs::PoseStamped transformedPoseWRTchild;
        auto required_transform = tf_buffer.lookupTransform(ChildLink, transformedPoseEachLink.header.frame_id, ros::Time(0));
        //auto transformedPose = calculateTransformStampPosition(transformedPoseEachLink, ChildLink);
        tf2::doTransform(transformedPoseEachLink.pose,transformedPoseWRTchild.pose,required_transform);
        
        tf2::Vector3 positionRelativeToCameraSensorLink;
        tf2::Quaternion quaternionRelativeToCameraLink;
	    tf2::fromMsg(transformedPoseWRTchild.pose.orientation, quaternionRelativeToCameraLink);
        tf2::fromMsg(transformedPoseWRTchild.pose.position, positionRelativeToCameraSensorLink);

        double roll, pitch, yaw;
        tf2::Matrix3x3 rotationRelativeToCameraSensorLink;
        rotationRelativeToCameraSensorLink.setRotation(quaternionRelativeToCameraLink);
        rotationRelativeToCameraSensorLink.getRPY(roll, pitch, yaw);
        tf::Matrix3x3 rotationCameraSensorLink;
        rotationCameraSensorLink.setRPY(roll, pitch, yaw);

       // auto translation = transformCameraSensorLink.getOrigin();
        auto rotationBody1ToJoint1 =   rotationCameraSensorLink;
        tf::Vector3 translationBody1ToJoin1 = {positionRelativeToCameraSensorLink[0], positionRelativeToCameraSensorLink[1], positionRelativeToCameraSensorLink[2]};

        geometry_msgs::PoseStamped transformedPoseWRTparent;
        required_transform = tf_buffer.lookupTransform(ParentLink, transformedPoseEachLink.header.frame_id, ros::Time(0));
        //auto transformedPose = calculateTransformStampPosition(transformedPoseEachLink, ChildLink);
        tf2::doTransform(transformedPoseEachLink.pose,transformedPoseWRTchild.pose,required_transform);

	    tf2::fromMsg(transformedPoseWRTchild.pose.orientation, quaternionRelativeToCameraLink);
        tf2::fromMsg(transformedPoseWRTchild.pose.position, positionRelativeToCameraSensorLink);

 
        rotationRelativeToCameraSensorLink.setRotation(quaternionRelativeToCameraLink);
        rotationRelativeToCameraSensorLink.getRPY(roll, pitch, yaw);

        rotationCameraSensorLink.setRPY(roll, pitch, yaw);

       // auto translation = transformCameraSensorLink.getOrigin();
        auto rotationBody2ToJoint2 =   rotationCameraSensorLink;
        tf::Vector3 translationBody2ToJoin2 = {positionRelativeToCameraSensorLink[0], positionRelativeToCameraSensorLink[1], positionRelativeToCameraSensorLink[2]};

    
        writeToConstraint(rotationBody1ToJoint1, rotationBody2ToJoint2, translationBody2ToJoin2, translationBody1ToJoin1, filename, body_id);

};

if(use_tf)
{

    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("base_link"), 0);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("shoulder_link"), 1);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("bicep_link"), 2);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("forearm_link"), 3);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("spherical_wrist_1_link"), 4);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("spherical_wrist_2_link"), 5);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("bracelet_link"), 6);

    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("robotiq_arg2f_base_link"), 7);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("left_inner_knuckle"), 8);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("left_outer_knuckle"), 9);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("left_outer_finger"), 10);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("left_inner_finger"), 11);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("left_inner_finger_pad"), 12);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("right_inner_knuckle"), 13);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("right_outer_knuckle"), 14);
    ArmLinkToCameraSensorLinkTF(tf_prefix + std::string("right_inner_finger_pad"), 15);

    ArmLinkToArmLinkTF(tf_prefix + std::string("base_link"), tf_prefix + std::string("shoulder_link"), 1);
    ArmLinkToArmLinkTF(tf_prefix + std::string("shoulder_link"), tf_prefix + std::string("bicep_link"), 2);
    ArmLinkToArmLinkTF(tf_prefix + std::string("bicep_link"), tf_prefix + std::string("forearm_link"), 3);
    ArmLinkToArmLinkTF(tf_prefix + std::string("forearm_link"), tf_prefix + std::string("spherical_wrist_1_link"), 4);
    ArmLinkToArmLinkTF(tf_prefix + std::string("spherical_wrist_1_link"), tf_prefix + std::string("spherical_wrist_2_link"), 5);
    ArmLinkToArmLinkTF(tf_prefix + std::string("spherical_wrist_2_link"), tf_prefix + std::string("bracelet_link"), 6);
    ArmLinkToArmLinkTF(tf_prefix + std::string("bracelet_link"), tf_prefix + std::string("robotiq_arg2f_base_link"), 7);
 /*   ArmLinkToArmLinkTF(tf_prefix + std::string("robotiq_arg2f_base_link"), tf_prefix + std::string("left_outer_knuckle"), 8);
    ArmLinkToArmLinkTF(tf_prefix + std::string("robotiq_arg2f_base_link"), tf_prefix + std::string("right_outer_knuckle"), 9);
    ArmLinkToArmLinkTF(tf_prefix + std::string("robotiq_arg2f_base_link"), tf_prefix + std::string("left_inner_knuckle"), 10);
    ArmLinkToArmLinkTF(tf_prefix + std::string("robotiq_arg2f_base_link"), tf_prefix + std::string("left_inner_knuckle"), 11);    
    ArmLinkToArmLinkTF(tf_prefix + std::string("left_outer_knuckle"), tf_prefix + std::string("left_outer_finger"), 12);
    ArmLinkToArmLinkTF(tf_prefix + std::string("right_outer_knuckle"), tf_prefix + std::string("right_outer_finger"), 13);  
    ArmLinkToArmLinkTF(tf_prefix + std::string("left_outer_finger"), tf_prefix + std::string("left_inner_finger"), 14);
    ArmLinkToArmLinkTF(tf_prefix + std::string("right_outer_finger"), tf_prefix + std::string("right_inner_finger"), 15);  
    ArmLinkToArmLinkTF(tf_prefix + std::string("left_inner_finger"), tf_prefix + std::string("left_inner_finger_pad"), 16);
    ArmLinkToArmLinkTF(tf_prefix + std::string("right_inner_finger"), tf_prefix + std::string("right_inner_finger_pad"), 17); 
******/
    writeToConstraints(tf_prefix + std::string("base_link"), tf_prefix + std::string("shoulder_link"),std::string("constraint_1_2.yaml"), 1);
    writeToConstraints(tf_prefix + std::string("shoulder_link"), tf_prefix + std::string("bicep_link"),std::string("constraint_2_3.yaml"), 2);
    writeToConstraints(tf_prefix + std::string("bicep_link"), tf_prefix + std::string("forearm_link"),std::string("constraint_3_4.yaml"), 3);
    writeToConstraints(tf_prefix + std::string("forearm_link"), tf_prefix + std::string("spherical_wrist_1_link"), std::string("constraint_4_5.yaml"), 4);
    writeToConstraints(tf_prefix + std::string("spherical_wrist_1_link"), tf_prefix + std::string("spherical_wrist_2_link"), std::string("constraint_5_6.yaml"), 5);
    writeToConstraints(tf_prefix + std::string("spherical_wrist_2_link"), tf_prefix + std::string("bracelet_link"), std::string("constraint_6_7.yaml"), 6);
}

if(!use_tf)
{
    ArmToCameraSensorLinkTransformGazebo(client_base_link, base_link_state, std::string("base_link"), 0, 0);
    ArmToCameraSensorLinkTransformGazebo(client_shoulder_link, shoulder_link_state, std::string("shoulder_link"), 1, 0);
    ArmToCameraSensorLinkTransformGazebo(client_bicep_link, bicep_link_state, std::string("bicep_link"), 2, 0);
    ArmToCameraSensorLinkTransformGazebo(client_forearm_link, forearm_link_state, std::string("forearm_link"), 3, 0);
    ArmToCameraSensorLinkTransformGazebo(client_spherical_wrist_1_link, spherical_wrist_1_link_state, std::string("spherical_wrist_1_link"), 4, 0);
    ArmToCameraSensorLinkTransformGazebo(client_spherical_wrist_2_link, spherical_wrist_2_link_state, std::string("spherical_wrist_2_link"), 5, 0);
    ArmToCameraSensorLinkTransformGazebo(client_bracelet_link, bracelet_link_state, std::string("bracelet_link"), 6, 0);

/*
    ArmLinkToArmLinkGazebo(client_shoulder_base, shoulder_base_state, std::string("base_link"), std::string("shoulder_link"));
    ArmLinkToArmLinkGazebo(client_bicep_shoulder, bicep_shoulder_state, std::string("shoulder_link"), std::string("bicep_link"));
    ArmLinkToArmLinkGazebo(client_forearm_bicep, forearm_bicep_state, std::string("bicep_link"), std::string("forearm_link"));
    ArmLinkToArmLinkGazebo(client_spherical_wrist_1_forearm, spherical_wrist_1_forearm_state, std::string("forearm_link"), std::string("spherical_wrist_1_link"));
    ArmLinkToArmLinkGazebo(client_spherical_wrist_2_spherical_wrist_1, spherical_wrist_2_spherical_wrist_1_state, std::string("spherical_wrist_1_link"), std::string("spherical_wrist_2_link"));
    ArmLinkToArmLinkGazebo(client_bracelet_spherical_wrist_2, bracelet_spherical_wrist_2_state, std::string("spherical_wrist_2_link"), std::string("bracelet_link"));
  */      
    ArmLinkToArmLinkTF(std::string("shoulder_link"), std::string("base_link"), 1);
    ArmLinkToArmLinkTF(std::string("bicep_link"), std::string("shoulder_link"), 2);
    ArmLinkToArmLinkTF(std::string("forearm_link"), std::string("bicep_link"), 3);
    ArmLinkToArmLinkTF(std::string("spherical_wrist_1_link"), std::string("forearm_link"), 4);
    ArmLinkToArmLinkTF(std::string("spherical_wrist_2_link"), std::string("spherical_wrist_1_link"), 5);
    ArmLinkToArmLinkTF(std::string("bracelet_link"), std::string("spherical_wrist_2_link"), 6);
}

auto writeCameraToWorld = [&]()
{
    tf::StampedTransform transform;
    tf::StampedTransform trans;
    try 
    {
     listener.lookupTransform(tf_prefix + "base_link", cameraSensorFrame, ros::Time(0), transform);
  
     auto rotation = transform.getBasis();

    auto translation = transform.getOrigin();

    writeToYamlStaticDetectorAndCameraToWord(rotation, tf2::Vector3{translation[0], translation[1], translation[2]}, true);

    } 
    catch (tf::TransformException &ex)
    {
        ROS_WARN ("%s", ex.what());
        ros::Duration (1.0).sleep();
    }
};

auto RPYtoMatrixConverter = []()
{
    tf::Matrix3x3 rotation;
    rotation.setRPY(1.57, 0, 0);
    tf::Vector3 translation{0, 0, 0};
    std::cout<< "["<<rotation[0][0]<< "," << rotation[0][1] << "," << rotation[0][2]
       << "," << translation[0] << ","
       << rotation[1][0] << "," << rotation[1][1] << "," << rotation[1][2]
       << "," << translation[1] << ","
       << rotation[2][0] << "," << rotation[2][1] << "," << rotation[2][2]
       << "," << translation[2] << ",0,0, 0,1]";

};
//RPYtoMatrixConverter();
//writeCameraToWorld();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "convertToMatrix");
  std::cout<<"reached here";
  ConvertToMatrix convertToMatrix;
  while(ros::ok())
  {
    convertToMatrix.configWriter();
  }

  ros::spin();
  return 0;
}