#include <ros/ros.h>
#include "rosneuro_recorder/Recorder.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "recorder");
	rosneuro::Recorder recorder;

	if(!recorder.Run()) {
        ROS_ERROR("Recorder interrupted while running");
    }

	ros::shutdown();
	return 0;
}
