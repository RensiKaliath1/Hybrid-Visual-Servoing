cmake_minimum_required (VERSION 3.5.1)
project(pose_estimation_icg)


# Options
# =============================================================================
option(USE_AZURE_KINECT "Use Azure Kinect" OFF)
option(USE_REALSENSE "Use RealSense D435" OFF)
option(USE_XTION "Use Xtion" ON)


# Libraries
# =============================================================================
if (WIN32)
    set(GLEW_STATIC true)
    set(OpenCV_STATIC true)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif ()

find_package(Eigen3 3.3.2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.1.2 REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)
if(USE_AZURE_KINECT)
    find_package(K4A 1.3.0 REQUIRED)
endif()
if (USE_REALSENSE)
    find_package(realsense2 REQUIRED)
endif ()

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    if (WIN32)
        set(CMAKE_CXX_FLAGS_RELEASE
                "${CMAKE_C_FLAGS_RELEASE} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS_RELEASE
                "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS
                "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    else ()
        set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS
                "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endIf ()
endif ()

# ROS components
# =============================================================================
find_package(catkin REQUIRED COMPONENTS
	dynamic_reconfigure
	geometry_msgs
	nav_msgs
	sensor_msgs
	std_msgs
	tf
	moveit_msgs
	image_transport
	cv_bridge
    estimated_pose_msg
    )

# ROS messages

catkin_package (
)


# Compiler Settings
# =============================================================================

# Default settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
# We are using Werror just to always display the warnings
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Werror")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-march HAS_MARCH)
check_cxx_compiler_flag(-mtune HAS_MTUNE)
if (HAS_MARCH)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif ()
if (HAS_MTUNE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")
endif ()
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wzero-as-null-pointer-constant")

# Debug settings
check_cxx_compiler_flag(-ggdb HAS_GGDB)
check_cxx_compiler_flag(-Z7 HAS_Z7)
check_cxx_compiler_flag(-ftrapv HAS_FTRAPV)
check_cxx_compiler_flag(-Od HAS_OD)
if (HAS_GGDB)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb")
elseif (HAS_Z7)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Z7")
endif ()
if (HAS_FTRAPV)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftrapv")
endif ()
if (HAS_OD)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Od")
endif ()


# Release settings
check_cxx_compiler_flag(-Ob3 HAS_OB3)
check_cxx_compiler_flag(-O3 HAS_O3)
check_cxx_compiler_flag(-Ob2 HAS_OB2)
check_cxx_compiler_flag(-O2 HAS_O2)
if (HAS_OB3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob3")
elseif (HAS_O3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
elseif (HAS_OB2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob2")
elseif (HAS_O2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

# OS settings
if (WIN32)
    if(OPENMP_FOUND)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -openmp:experimental")
    endif()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -MTd")
endif()


# Definitions
# =============================================================================
if (USE_AZURE_KINECT)
    add_definitions( -DUSE_AZURE_KINECT=${USE_AZURE_KINECT} )
endif ()
if (USE_REALSENSE)
    add_definitions( -DUSE_REALSENSE=${USE_REALSENSE} )
endif ()


# Directories
# =============================================================================
include_directories (
	${catkin_INCLUDE_DIRS}
	include
    include/camera_viewer
	third_party
	${OpenCV_INCLUDE_DIRS}
)


# Define sources
# =============================================================================
set(SOURCES
        src/main_ros.cpp
        src/MainRos.cpp
        src/ros_camera.cpp
        src/common.cpp 
        src/body.cpp 
        src/renderer_geometry.cpp 
        src/renderer.cpp
        src/normal_renderer.cpp 
        src/basic_depth_renderer.cpp 
        src/model.cpp src/region_model.cpp
        src/depth_model.cpp 
        src/camera.cpp 
        src/loader_camera.cpp 
        src/viewer.cpp
        src/image_viewer.cpp 
        src/normal_viewer.cpp 
        src/modality.cpp 
        src/region_modality.cpp
        src/depth_modality.cpp 
        src/optimizer.cpp 
        src/detector.cpp 
        src/static_detector.cpp
        src/manual_detector.cpp 
        src/refiner.cpp 
        src/publisher.cpp 
        src/tracker.cpp
        src/generator.cpp
        src/ros_publisher.cpp)

if (USE_AZURE_KINECT)
    list(APPEND SOURCES azure_kinect_camera.cpp)
endif ()
if (USE_REALSENSE)
    list(APPEND SOURCES realsense_camera.cpp)
endif ()


# Define libraries
# =============================================================================
set(LIBRARIES Eigen3::Eigen OpenGL::GL GLEW::GLEW glfw ${OpenCV_LIBRARIES})
if (USE_AZURE_KINECT)
    list(APPEND LIBRARIES k4a::k4a)
endif ()
if (USE_REALSENSE)
    list(APPEND LIBRARIES ${realsense2_LIBRARY})
endif ()
if(UNIX)
    list(APPEND LIBRARIES pthread stdc++fs)
endif()


# Executables
# =============================================================================
add_executable (posetracking_icg ${SOURCES})
target_link_libraries (posetracking_icg ${catkin_LIBRARIES} ${OpenCV_LIBS} ${LIBRARIES})
# target_compile_options(posetracking PUBLIC -Wall -Wextra -Werror -Wconversion -Wsign-conversion)
