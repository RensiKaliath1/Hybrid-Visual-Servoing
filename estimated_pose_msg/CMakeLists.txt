cmake_minimum_required(VERSION 3.1.3)
project(estimated_pose_msg)

find_package(catkin REQUIRED
    message_generation
    std_msgs
    geometry_msgs
    )

add_message_files(DIRECTORY msg FILES estimatedPose.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)


catkin_package()
