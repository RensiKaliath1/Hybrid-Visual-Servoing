#include <ibvs_arm.h>

int main (int argc, char *argv[])
 {
	ros::init(argc, argv, "ibv_surf");
    ros::NodeHandle nh;
	ros::CallbackQueue callbackQueue;

	ros::AsyncSpinner spinner (1, &callbackQueue);
	nh.setCallbackQueue (&callbackQueue);
	ROS_INFO ("Starting node ...\n");
    // moveIt uses the default queue now and the rest the async CB queue
    spinner.start();
	IBVS ibvs(nh);
	ros::spin();
	return 0;
}