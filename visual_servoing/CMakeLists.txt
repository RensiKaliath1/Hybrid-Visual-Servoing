cmake_minimum_required (VERSION 3.16.3)
project (visual_servoing)

#~ set (CMAKE_CXX_STANDARD 17)
#~ if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	#~ add_compile_options (-Wall -Wno-register)
#~ endif ()

add_compile_options ("-Wall" "-std=c++17")

list (APPEND CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
include (CommonMacros)

#~ find_package (OpenCV REQUIRED)

#list (PREPEND CMAKE_PREFIX_PATH /scratch-local/cdtemp/rensi/install/visp-3.6.0)
require_custom_package (OpenCV 4)
find_package(VISP 3.6.0 PATHS /scratch-local/cdtemp/rensi/install/visp-3.6.0 REQUIRED visp_core visp_robot visp_vs visp_blob visp_vision visp_io visp_gui visp_ar visp_mbt visp_sensor visp_detection )
#~ find_package(VISP REQUIRED visp_core visp_robot visp_vs visp_blob visp_vision visp_io visp_gui visp_ar visp_mbt visp_sensor visp_detection )
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(gazebo REQUIRED)

find_package (catkin REQUIRED COMPONENTS
	actionlib_msgs
	actionlib
	roscpp
	cv4_bridge
	control_msgs
	image_transport
	camera_calibration_parsers
  	geometry_msgs
  	sensor_msgs
  	visualization_msgs
  	std_msgs
  	actionlib
	control_msgs
	#dynamixel_msgs
	controller_manager
	moveit_core
	moveit_ros_move_group
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_visual_tools
	tf
	tf2
	tf_conversions
	tf2_geometry_msgs
	gazebo_msgs
	gazebo_ros
	estimated_pose_msg
   message_generation
   kinova_twist
   kortex_driver
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  points_msg.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
     std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    LIBRARIES arm_controller_lib

  LIBRARIES
    ${PROJECT_NAME}

  CATKIN_DEPENDS
    camera_calibration_parsers
    geometry_msgs
    visualization_msgs
    roscpp
    sensor_msgs
    std_msgs
	actionlib_msgs
  DEPENDS
    Boost
    VISP
    EIGEN3
)

# set the list of source files
set(tutorial_cpp
  tutorial-ibvs-4pts.cpp
  tutorial-ibvs-4pts-display.cpp
  tutorial-ibvs-4pts-image-tracking.cpp
  tutorial-ibvs-4pts-ogre.cpp
  tutorial-ibvs-4pts-ogre-tracking.cpp
  tutorial-ibvs-4pts-plotter.cpp
  tutorial-ibvs-4pts-plotter-gain-adaptive.cpp
  tutorial-ibvs-4pts-wireframe-camera.cpp
  tutorial-ibvs-4pts-wireframe-robot-afma6.cpp
  tutorial-ibvs-4pts-wireframe-robot-viper.cpp
  tutorial-ibvs-4pts-json.cpp

)

if(NOT DEFINED WINRT_8_1)
  list(APPEND tutorial_cpp tutorial-ibvs-4pts-plotter-continuous-gain-adaptive.cpp)
endif()

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/sphere/Sphere.mesh" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/sphere/Sphere.material" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/model_images/target_square.pgm" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/src/asymmetric_pipestar_with_cap_manual_detector.png" )

# ----------------------------------------------------------------------
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories (
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${VISP_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	third_party
	${Boost_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

add_executable ( ibvs_arm src/ibvs_arm_node src/ibvs_arm.cpp src/ibvs_keypoint_tracker.cpp )
target_link_libraries (ibvs_arm  ${catkin_LIBRARIES}	${OpenCV_LIBRARIES}
${VISP_LIBRARIES} ${GAZEBO_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})

add_executable ( pbvs src/pbvs_node.cpp src/pbvs.cpp )
target_link_libraries (pbvs  ${catkin_LIBRARIES}	${OpenCV_LIBRARIES}
${VISP_LIBRARIES} ${GAZEBO_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
