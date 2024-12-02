/** Written by the Object Handling subgroup
 **/

// #include "MainRos.cpp"
#include <icg/MainRos.h>

int main (int argc, char *argv[])
{
    ros::init (argc, argv, "posetracker");
    ros::NodeHandle nh;
    
    ros::CallbackQueue callbackQueue;
    nh.setCallbackQueue (&callbackQueue);

    ROS_INFO ("Starting node ...\n");

    RosICGTracker icgTracker = RosICGTracker (&nh);

    ros::AsyncSpinner spinner (4, &callbackQueue);
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO ("Done\n");

    std::cout << "GOODBYE ..." << std::endl;
}