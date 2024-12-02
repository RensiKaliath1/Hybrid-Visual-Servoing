cmake_minimum_required(VERSION 3.1.3)
project(kinova_twist)

find_package(catkin REQUIRED
    message_generation
    std_msgs
    geometry_msgs
    )

add_message_files(DIRECTORY msg FILES Twist.msg TwistCommand.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)


catkin_package()
