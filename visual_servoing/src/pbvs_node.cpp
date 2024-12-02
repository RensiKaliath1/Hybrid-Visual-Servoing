#include "pbvs.h"

int main (int argc, char *argv[])
{
	ros::init(argc, argv, "pbvs");
	PBVS PBVS;
	ros::spin();
	return 0;
}